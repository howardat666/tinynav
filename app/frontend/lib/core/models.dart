import 'dart:convert';
import 'dart:typed_data';

class NavProgress {
  final int poiIndex;
  final double percent;
  final double pathRemainingM;
  final double pathTotalM;
  final double estimatedRemainingS;

  const NavProgress({
    required this.poiIndex,
    required this.percent,
    required this.pathRemainingM,
    required this.pathTotalM,
    required this.estimatedRemainingS,
  });

  factory NavProgress.fromJson(Map<String, dynamic> json) => NavProgress(
        poiIndex: json['poi_index'] as int? ?? 0,
        percent: (json['percent'] as num?)?.toDouble() ?? 0.0,
        pathRemainingM: (json['path_remaining_m'] as num?)?.toDouble() ?? 0.0,
        pathTotalM: (json['path_total_m'] as num?)?.toDouble() ?? 0.0,
        estimatedRemainingS: (json['estimated_remaining_s'] as num?)?.toDouble() ?? -1.0,
      );
}

/// One relocalisation counting window, as map_node reports it.
class RelocWindow {
  final int keyframes;
  final int attempts;
  final int success;
  final int droppedStale;
  final int skippedRateLimit;
  final double? successRate;
  final double attemptHz;
  final double successHz;
  final double spanS;
  final Map<String, int> byCode;

  const RelocWindow({
    required this.keyframes,
    required this.attempts,
    required this.success,
    required this.droppedStale,
    required this.skippedRateLimit,
    required this.successRate,
    required this.attemptHz,
    required this.successHz,
    required this.spanS,
    required this.byCode,
  });

  /// Keyframes per second, which is what says whether the front end is feeding
  /// map_node fast enough. Reported as a count and a span, never as a rate.
  double get keyframeHz => spanS > 0 ? keyframes / spanS : 0.0;

  factory RelocWindow.fromJson(Map<String, dynamic> j) => RelocWindow(
        keyframes: (j['keyframes'] as num?)?.toInt() ?? 0,
        attempts: (j['attempts'] as num?)?.toInt() ?? 0,
        success: (j['success'] as num?)?.toInt() ?? 0,
        droppedStale: (j['dropped_stale'] as num?)?.toInt() ?? 0,
        skippedRateLimit: (j['skipped_rate_limit'] as num?)?.toInt() ?? 0,
        successRate: (j['successRate'] as num?)?.toDouble(),
        attemptHz: (j['attemptHz'] as num?)?.toDouble() ?? 0.0,
        successHz: (j['successHz'] as num?)?.toDouble() ?? 0.0,
        spanS: (j['spanS'] as num?)?.toDouble() ?? 0.0,
        byCode: (j['byCode'] as Map?)?.map(
              (k, v) => MapEntry(k.toString(), (v as num).toInt()),
            ) ??
            const <String, int>{},
      );
}

class RelocStats {
  final RelocWindow? window;
  final RelocWindow? total;
  final String? lastFailureCode;
  final String? lastFailureReason;
  final double? secondsSinceLastSuccess;

  const RelocStats({
    this.window,
    this.total,
    this.lastFailureCode,
    this.lastFailureReason,
    this.secondsSinceLastSuccess,
  });

  factory RelocStats.fromJson(Map<String, dynamic> j) => RelocStats(
        window: j['window'] == null
            ? null
            : RelocWindow.fromJson(j['window'] as Map<String, dynamic>),
        total: j['total'] == null
            ? null
            : RelocWindow.fromJson(j['total'] as Map<String, dynamic>),
        lastFailureCode: j['lastFailureCode'] as String?,
        lastFailureReason: j['lastFailureReason'] as String?,
        secondsSinceLastSuccess: (j['secondsSinceLastSuccess'] as num?)?.toDouble(),
      );
}

class DeviceStatus {
  final bool online;
  final double? battery;
  final String bagStatus;
  final bool bagFileReady;
  final String mapStatus;
  final double mappingPercent;
  final String navStatus;
  final String rawState;
  final bool navNodesRunning;
  final bool navPaused;
  final RelocStats? reloc;

  const DeviceStatus({
    required this.online,
    this.battery,
    required this.bagStatus,
    required this.bagFileReady,
    required this.mapStatus,
    required this.mappingPercent,
    required this.navStatus,
    required this.rawState,
    required this.navNodesRunning,
    required this.navPaused,
    this.reloc,
  });

  factory DeviceStatus.fromJson(Map<String, dynamic> json) => DeviceStatus(
        online: json['online'] as bool? ?? false,
        battery: (json['battery'] as num?)?.toDouble(),
        bagStatus: json['bagStatus'] as String? ?? 'idle',
        bagFileReady: json['bagFileReady'] as bool? ?? false,
        mapStatus: json['mapStatus'] as String? ?? 'idle',
        mappingPercent: (json['mappingPercent'] as num?)?.toDouble() ?? 0.0,
        navStatus: json['navStatus'] as String? ?? 'idle',
        rawState: json['rawState'] as String? ?? 'unknown',
        navNodesRunning: json['navNodesRunning'] as bool? ?? false,
        navPaused: json['navPaused'] as bool? ?? false,
        reloc: json['relocalization'] == null
            ? null
            : RelocStats.fromJson(json['relocalization'] as Map<String, dynamic>),
      );
}

class Pose {
  final double x;
  final double y;
  final double yaw;
  final double? z;
  final double? timestamp;

  const Pose({required this.x, required this.y, required this.yaw, this.z, this.timestamp});

  factory Pose.fromJson(Map<String, dynamic> json) => Pose(
        x: (json['x'] as num).toDouble(),
        y: (json['y'] as num).toDouble(),
        yaw: (json['yaw'] as num).toDouble(),
        z: (json['z'] as num?)?.toDouble(),
        timestamp: (json['timestamp'] as num?)?.toDouble(),
      );
}

class MapInfo {
  final String imageUrl;
  final double originX;
  final double originY;
  final double resolution;
  final int width;
  final int height;

  const MapInfo({
    required this.imageUrl,
    required this.originX,
    required this.originY,
    required this.resolution,
    required this.width,
    required this.height,
  });

  factory MapInfo.fromJson(Map<String, dynamic> json) => MapInfo(
        imageUrl: json['imageUrl'] as String,
        originX: (json['origin_x'] as num).toDouble(),
        originY: (json['origin_y'] as num).toDouble(),
        resolution: (json['resolution'] as num).toDouble(),
        width: json['width'] as int,
        height: json['height'] as int,
      );
}

class MapFileInfo {
  final String imageUrl;
  final double originX;
  final double originY;
  final double resolution;
  final int width;
  final int height;
  final List<Poi> pois;

  const MapFileInfo({
    required this.imageUrl,
    required this.originX,
    required this.originY,
    required this.resolution,
    required this.width,
    required this.height,
    required this.pois,
  });

  factory MapFileInfo.fromJson(Map<String, dynamic> json) => MapFileInfo(
        imageUrl: json['imageUrl'] as String,
        originX: (json['origin_x'] as num).toDouble(),
        originY: (json['origin_y'] as num).toDouble(),
        resolution: (json['resolution'] as num).toDouble(),
        width: json['width'] as int,
        height: json['height'] as int,
        pois: (json['pois'] as List)
            .map((p) => Poi.fromJson(p as Map<String, dynamic>))
            .toList(),
      );
}

class TrajPoint {
  final double x;
  final double y;
  const TrajPoint(this.x, this.y);
}

class VoxelPoint {
  final double x;
  final double y;
  final double z;
  const VoxelPoint(this.x, this.y, this.z);
}

class GridInfo {
  final double originX;
  final double originY;
  final double resolution;
  final int width;
  final int height;

  const GridInfo({
    required this.originX,
    required this.originY,
    required this.resolution,
    required this.width,
    required this.height,
  });

  factory GridInfo.fromJson(Map<String, dynamic> j) => GridInfo(
        originX: (j['origin_x'] as num).toDouble(),
        originY: (j['origin_y'] as num).toDouble(),
        resolution: (j['resolution'] as num).toDouble(),
        width: j['width'] as int,
        height: j['height'] as int,
      );
}

/// planning_node's per-cycle numbers, published at 2 Hz for the diagnostics panel.
class PlanningDiag {
  final double? frontClearanceM;
  final bool frontBlocked;
  final double frontBlockedAtM;
  final double frontProbeMaxM;
  final int obstacleCells;
  final double? esdfAtRobotM;
  final double? cycleS;
  final double? stampLagS;

  const PlanningDiag({
    this.frontClearanceM,
    required this.frontBlocked,
    required this.frontBlockedAtM,
    required this.frontProbeMaxM,
    required this.obstacleCells,
    this.esdfAtRobotM,
    this.cycleS,
    this.stampLagS,
  });

  factory PlanningDiag.fromJson(Map<String, dynamic> j) => PlanningDiag(
        frontClearanceM: (j['frontClearanceM'] as num?)?.toDouble(),
        frontBlocked: j['frontBlocked'] as bool? ?? false,
        frontBlockedAtM: (j['frontBlockedAtM'] as num?)?.toDouble() ?? 0.0,
        frontProbeMaxM: (j['frontProbeMaxM'] as num?)?.toDouble() ?? 0.5,
        obstacleCells: (j['obstacleCells'] as num?)?.toInt() ?? 0,
        esdfAtRobotM: (j['esdfAtRobotM'] as num?)?.toDouble(),
        cycleS: (j['cycleS'] as num?)?.toDouble(),
        stampLagS: (j['stampLagS'] as num?)?.toDouble(),
      );
}

class PlanningState {
  final bool localized;
  final Pose? odomPose;
  final Pose? odomPoseAtKf;
  final Pose? mapPose;
  final Uint8List? esdfImage;
  final Uint8List? obstacleImage;
  final List<TrajPoint> trajectory;
  final List<TrajPoint> globalPath;
  final List<TrajPoint> mapGlobalPath;
  final GridInfo? gridInfo;
  final TrajPoint? navTargetPose;
  final List<TrajPoint> footprint;
  final List<VoxelPoint> voxelPoints;
  final PlanningDiag? diag;

  const PlanningState({
    required this.localized,
    this.odomPose,
    this.odomPoseAtKf,
    this.mapPose,
    this.esdfImage,
    this.obstacleImage,
    required this.trajectory,
    required this.globalPath,
    this.mapGlobalPath = const [],
    this.gridInfo,
    this.navTargetPose,
    this.footprint = const [],
    this.voxelPoints = const [],
    this.diag,
  });

  factory PlanningState.fromJson(Map<String, dynamic> j) {
    Uint8List? decodeImg(String? b64) {
      if (b64 == null || b64.isEmpty) return null;
      return base64Decode(b64);
    }

    Pose? parsePose(Object? raw) {
      if (raw == null) return null;
      return Pose.fromJson(raw as Map<String, dynamic>);
    }

    List<TrajPoint> parsePath(String key) =>
        (j[key] as List? ?? []).map((p) {
          final m = p as Map<String, dynamic>;
          return TrajPoint((m['x'] as num).toDouble(), (m['y'] as num).toDouble());
        }).toList();

    return PlanningState(
      localized: j['localized'] as bool? ?? false,
      odomPose: parsePose(j['odom_pose']),
      odomPoseAtKf: parsePose(j['odom_pose_at_kf']),
      mapPose: parsePose(j['map_pose']),
      esdfImage: decodeImg(j['esdf_image'] as String?),
      obstacleImage: decodeImg(j['obstacle_image'] as String?),
      trajectory: parsePath('trajectory'),
      globalPath: parsePath('global_path'),
      mapGlobalPath: parsePath('map_global_path'),
      gridInfo: j['grid_info'] != null
          ? GridInfo.fromJson(j['grid_info'] as Map<String, dynamic>)
          : null,
      navTargetPose: j['nav_target_pose'] != null
          ? TrajPoint(
              (j['nav_target_pose']['x'] as num).toDouble(),
              (j['nav_target_pose']['y'] as num).toDouble(),
            )
          : null,
      footprint: (j['footprint'] as List? ?? []).map((p) {
        final m = p as Map<String, dynamic>;
        return TrajPoint((m['x'] as num).toDouble(), (m['y'] as num).toDouble());
      }).toList(),
      voxelPoints: (j['voxel_points'] as List? ?? []).map((p) {
        final m = p as Map<String, dynamic>;
        return VoxelPoint(
          (m['x'] as num).toDouble(),
          (m['y'] as num).toDouble(),
          (m['z'] as num).toDouble(),
        );
      }).toList(),
      diag: j['diag'] == null
          ? null
          : PlanningDiag.fromJson(j['diag'] as Map<String, dynamic>),
    );
  }
}

class SysInfo {
  final double cpuPercent;
  final double memPercent;
  final double memUsedGb;
  final double memTotalGb;
  final double diskPercent;
  final double diskUsedGb;
  final double diskTotalGb;
  final double? gpuPercent;

  const SysInfo({
    required this.cpuPercent,
    required this.memPercent,
    required this.memUsedGb,
    required this.memTotalGb,
    required this.diskPercent,
    required this.diskUsedGb,
    required this.diskTotalGb,
    this.gpuPercent,
  });

  factory SysInfo.fromJson(Map<String, dynamic> j) => SysInfo(
        cpuPercent: (j['cpu_percent'] as num).toDouble(),
        memPercent: (j['mem_percent'] as num).toDouble(),
        memUsedGb: (j['mem_used_gb'] as num).toDouble(),
        memTotalGb: (j['mem_total_gb'] as num).toDouble(),
        diskPercent: (j['disk_percent'] as num).toDouble(),
        diskUsedGb: (j['disk_used_gb'] as num).toDouble(),
        diskTotalGb: (j['disk_total_gb'] as num).toDouble(),
        gpuPercent: (j['gpu_percent'] as num?)?.toDouble(),
      );
}

class FileEntry {
  final String name;
  final int size;
  final double mtime;
  final bool isDir;

  const FileEntry({
    required this.name,
    required this.size,
    required this.mtime,
    required this.isDir,
  });

  factory FileEntry.fromJson(Map<String, dynamic> j) => FileEntry(
        name: j['name'] as String,
        size: (j['size'] as num).toInt(),
        mtime: (j['mtime'] as num).toDouble(),
        isDir: j['is_dir'] as bool? ?? false,
      );

  String get sizeLabel {
    if (size < 1024) return '${size}B';
    if (size < 1024 * 1024) return '${(size / 1024).toStringAsFixed(1)}KB';
    if (size < 1024 * 1024 * 1024) {
      return '${(size / (1024 * 1024)).toStringAsFixed(1)}MB';
    }
    return '${(size / (1024 * 1024 * 1024)).toStringAsFixed(2)}GB';
  }
}

class Poi {
  final int id;
  final String name;
  final double x;
  final double y;
  final double z;

  const Poi({
    required this.id,
    required this.name,
    required this.x,
    required this.y,
    required this.z,
  });

  factory Poi.fromJson(Map<String, dynamic> json) {
    final pos = json['position'] as List;
    return Poi(
      id: json['id'] as int,
      name: json['name'] as String,
      x: (pos[0] as num).toDouble(),
      y: (pos[1] as num).toDouble(),
      z: (pos[2] as num).toDouble(),
    );
  }
}
