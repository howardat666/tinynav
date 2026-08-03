// Minimal pybind11 bindings for DBoW3.
//
// The upstream pyDBoW3 bindings carry a vendored numpy<->cv::Mat converter that
// no longer compiles against OpenCV 4.x (cv::MatAllocator gained pure-virtual
// overloads, so its NumpyAllocator is abstract). We only ever pass dense
// (N, 32) uint8 ORB descriptors, so a direct wrap is both smaller and portable.
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

using U8Array = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;

static cv::Mat to_mat(const U8Array &array) {
    const py::buffer_info info = array.request();
    if (info.ndim != 2) {
        throw std::runtime_error("descriptors must be 2-D (N, 32) uint8");
    }
    // DBoW3 keeps a reference to the rows, so hand it an owned copy.
    return cv::Mat(static_cast<int>(info.shape[0]),
                   static_cast<int>(info.shape[1]),
                   CV_8UC1,
                   info.ptr).clone();
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
        .def("create", [](DBoW3::Vocabulary &self, const std::vector<U8Array> &features) {
            std::vector<cv::Mat> training;
            training.reserve(features.size());
            for (const auto &f : features) {
                cv::Mat mat = to_mat(f);
                if (mat.rows > 0) {
                    training.push_back(mat);
                }
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
        .def("add", [](DBoW3::Database &self, const U8Array &descriptors) {
            return static_cast<int>(self.add(to_mat(descriptors)));
        })
        .def("query", [](DBoW3::Database &self, const U8Array &descriptors, int max_results) {
            DBoW3::QueryResults results;
            self.query(to_mat(descriptors), results, max_results);
            std::vector<QueryResult> out;
            out.reserve(results.size());
            for (const auto &r : results) {
                out.push_back({r.Id, r.Score});
            }
            return out;
        }, py::arg("descriptors"), py::arg("max_results") = 10)
        .def("size", [](const DBoW3::Database &self) { return self.size(); })
        .def("clear", [](DBoW3::Database &self) { self.clear(); });
}
