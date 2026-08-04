// Minimal pybind11 bindings for DBoW3.
//
// The upstream pyDBoW3 bindings carry a vendored numpy<->cv::Mat converter that
// no longer compiles against OpenCV 4.x (cv::MatAllocator gained pure-virtual
// overloads, so its NumpyAllocator is abstract). We only pass dense descriptor
// blocks, so a direct wrap is both smaller and portable.
//
// Two descriptor layouts are supported, matching the two branches DBoW3's
// DescManip already implements:
//   * (N, 32) uint8   -> CV_8U,  Hamming distance   (ORB)
//   * (N, D)  float32 -> CV_32F, L2 distance        (SuperPoint, D = 256)
// Dispatch is on the array's dtype. Note there is deliberately no
// py::array::forcecast here: with forcecast a float32 array is silently
// truncated to uint8 (SuperPoint descriptors in [0, 1) all become zeros) and
// DBoW3 happily trains on the garbage without ever raising.
//
// API is kept source-compatible with pyDBoW3 as used by
// tinynav/core/models_trt.py::DBoW3Engine.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <opencv2/core.hpp>
#include <DBoW3/DBoW3.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

// c_style without forcecast: a non-contiguous array of the right dtype is
// copied, an array of the wrong dtype is refused.
using U8Array = py::array_t<uint8_t, py::array::c_style>;
using F32Array = py::array_t<float, py::array::c_style>;

static cv::Mat mat_from(const py::array &array, int cv_type) {
    const py::buffer_info info = array.request();
    if (info.ndim != 2) {
        throw std::runtime_error("descriptors must be a 2-D (N, D) array, got " +
                                 std::to_string(info.ndim) + "-D");
    }
    const int rows = static_cast<int>(info.shape[0]);
    const int cols = static_cast<int>(info.shape[1]);
    if (cv_type == CV_8UC1 && cols % 8 != 0) {
        // DescManip's Hamming path reads the rows as uint64_t words.
        throw std::runtime_error("uint8 descriptor width must be a multiple of 8 "
                                 "(ORB is 32), got " + std::to_string(cols));
    }
    // DBoW3 keeps a reference to the rows, so hand it an owned copy.
    return cv::Mat(rows, cols, cv_type, info.ptr).clone();
}

template <typename ArrayT>
static cv::Mat as_contiguous_mat(const py::array &array, int cv_type) {
    // ensure() re-orders a Fortran/strided array into a C-contiguous copy; it
    // never changes the dtype, since ArrayT does not carry forcecast.
    ArrayT contiguous = ArrayT::ensure(array);
    if (!contiguous) {
        throw std::runtime_error("could not make the descriptor array C-contiguous");
    }
    return mat_from(contiguous, cv_type);
}

static cv::Mat to_mat(const py::array &array) {
    // Read kind/itemsize through the numpy dtype object rather than
    // py::isinstance<array_t<...>>: the latter also tests the c_style flag, so
    // it rejects a Fortran-ordered uint8 array instead of letting ensure()
    // repack it. Going through attributes also keeps this working across the
    // pybind11 versions in play (2.9.1 on the aarch64 image).
    const py::object dtype = array.attr("dtype");
    const std::string kind = dtype.attr("kind").cast<std::string>();
    const ssize_t itemsize = dtype.attr("itemsize").cast<ssize_t>();

    if (kind == "u" && itemsize == 1) {
        return as_contiguous_mat<U8Array>(array, CV_8UC1);
    }
    if (kind == "f" && itemsize == 4) {
        return as_contiguous_mat<F32Array>(array, CV_32FC1);
    }
    throw std::runtime_error(
        "descriptors must be uint8 (Hamming, e.g. ORB (N, 32)) or float32 "
        "(L2, e.g. SuperPoint (N, 256)); got dtype " +
        std::string(py::str(array.dtype())) +
        " -- convert explicitly, this binding does not cast");
}

struct QueryResult {
    unsigned int id;
    double score;
};

PYBIND11_MODULE(pydbow3, m) {
    m.doc() = "Minimal DBoW3 bindings (numpy uint8 descriptors only)";

    py::class_<DBoW3::Vocabulary>(m, "Vocabulary")
        .def(py::init<>())
        // k^L words. The stock ORBvoc is k=10, L=6 (~1M words, ~475 MB
        // resident); a vocabulary trained on the target map can be far smaller.
        .def(py::init([](int k, int levels) {
            return new DBoW3::Vocabulary(k, levels, DBoW3::TF_IDF, DBoW3::L1_NORM);
        }), py::arg("k") = 10, py::arg("levels") = 6)
        .def("create", [](DBoW3::Vocabulary &self, const std::vector<py::array> &features) {
            std::vector<cv::Mat> training;
            training.reserve(features.size());
            for (const auto &f : features) {
                cv::Mat mat = to_mat(f);
                if (mat.rows == 0) {
                    continue;
                }
                // DBoW3 only looks at descriptors[0].type() and asserts on the
                // rest, so reject a mixed batch here with a readable message.
                if (!training.empty() && mat.type() != training.front().type()) {
                    throw std::runtime_error(
                        "all training descriptors must share one dtype "
                        "(all uint8 or all float32)");
                }
                training.push_back(mat);
            }
            if (training.empty()) {
                throw std::runtime_error("no training descriptors");
            }
            self.create(training);
        }, py::arg("training_features"))
        .def("load", [](DBoW3::Vocabulary &self, const std::string &path) {
            self.load(path);
            if (self.empty()) {
                throw std::runtime_error("vocabulary is empty after loading " + path);
            }
            return true;
        })
        .def("save", [](DBoW3::Vocabulary &self, const std::string &path) { self.save(path); })
        .def("size", [](const DBoW3::Vocabulary &self) { return self.size(); })
        .def("empty", [](const DBoW3::Vocabulary &self) { return self.empty(); });

    py::class_<QueryResult>(m, "QueryResult")
        // pyDBoW3 exposes capitalised names; keep both spellings working.
        .def_readonly("Id", &QueryResult::id)
        .def_readonly("id", &QueryResult::id)
        .def_readonly("Score", &QueryResult::score)
        .def_readonly("score", &QueryResult::score)
        .def("__repr__", [](const QueryResult &r) {
            return "<QueryResult Id=" + std::to_string(r.id) +
                   " Score=" + std::to_string(r.score) + ">";
        });

    py::class_<DBoW3::Database>(m, "Database")
        .def(py::init<>())
        .def("setVocabulary", [](DBoW3::Database &self, DBoW3::Vocabulary &voc) {
            self.setVocabulary(voc, false, 0);
        })
        .def("add", [](DBoW3::Database &self, const py::array &descriptors) {
            return static_cast<int>(self.add(to_mat(descriptors)));
        })
        .def("query", [](DBoW3::Database &self, const py::array &descriptors, int max_results) {
            DBoW3::QueryResults results;
            self.query(to_mat(descriptors), results, max_results);
            std::vector<QueryResult> out;
            out.reserve(results.size());
            for (const auto &r : results) {
                out.push_back({r.Id, r.Score});
            }
            return out;
        }, py::arg("descriptors"), py::arg("max_results") = 10)
        // Database::save() writes the vocabulary into the same cv::FileStorage
        // (Database.cpp calls m_voc->save(fs) first), so the file is
        // self-contained: load() below needs no prior setVocabulary(), and the
        // separate Vocabulary.load() can be skipped entirely.
        //
        // The extension picks the cv::FileStorage backend: ".yml"/".yaml" and
        // ".xml"/".json" are all text, and appending ".gz" gzips them.
        .def("save", [](const DBoW3::Database &self, const std::string &path) {
            // DBoW3 throws a bare std::string (not a std::exception) when
            // FileStorage cannot open the path; pybind11 has no translator for
            // that, so it would abort the interpreter instead of raising.
            try {
                self.save(path);
            } catch (const std::string &err) {
                throw std::runtime_error(err);
            }
        }, py::arg("path"))
        .def("load", [](DBoW3::Database &self, const std::string &path) {
            try {
                self.load(path);
            } catch (const std::string &err) {
                throw std::runtime_error(err);
            }
            // A file that parses but is not a database leaves the embedded
            // vocabulary empty rather than failing, which would then make every
            // query silently return nothing.
            const DBoW3::Vocabulary *voc = self.getVocabulary();
            if (voc == nullptr || voc->empty()) {
                throw std::runtime_error(
                    "database has no vocabulary after loading " + path);
            }
            return true;
        }, py::arg("path"))
        .def("size", [](const DBoW3::Database &self) { return self.size(); })
        .def("usingDirectIndex", [](const DBoW3::Database &self) {
            return self.usingDirectIndex();
        })
        .def("clear", [](DBoW3::Database &self) { self.clear(); });
}
