import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../core/models.dart';

/// Renders robot arrow, local trajectory, and global path on the local planning canvas.
/// The canvas maps to the planning grid: robot is always at center.
/// Global path arrives pre-converted to odom frame by the backend.
class LocalPlanningPainter extends CustomPainter {
  final List<TrajPoint> trajectory;
  final List<TrajPoint> globalPath;
  final List<TrajPoint> footprint;
  final Chassis? chassis;
  final GridInfo? gridInfo;
  final Pose? odomPose;
  final bool showTrajectory;
  final bool showGlobalPath;
  final bool showFootprint;
  final TrajPoint? navTargetPose;

  const LocalPlanningPainter({
    required this.trajectory,
    this.globalPath = const [],
    this.footprint = const [],
    this.chassis,
    this.gridInfo,
    this.odomPose,
    this.showTrajectory = true,
    this.showGlobalPath = true,
    this.showFootprint = true,
    this.navTargetPose,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;

    final gi = gridInfo;
    final pose = odomPose;

    // World coverage of the grid in meters (default 10 m × 10 m).
    final worldW = gi != null ? gi.width * gi.resolution : 10.0;
    final worldH = gi != null ? gi.height * gi.resolution : 10.0;
    final scaleX = size.width / worldW;
    final scaleY = size.height / worldH;

    if (showGlobalPath) _drawGlobalPath(canvas, cx, cy, scaleX, scaleY, pose);

    if (showTrajectory) _drawTrajectory(canvas, cx, cy, scaleX, scaleY, pose);

    if (navTargetPose != null && pose != null)
      _drawNavTarget(canvas, cx, cy, scaleX, scaleY, pose, navTargetPose!);

    if (showFootprint) _drawFootprint(canvas, cx, cy, scaleX, scaleY, pose);

    // Small arrow on top of everything
    _drawRobotArrow(canvas, Offset(cx, cy), pose?.yaw ?? 0.0);
  }

  void _drawTrajectory(Canvas canvas, double cx, double cy,
      double scaleX, double scaleY, Pose? pose) {
    if (trajectory.length < 2 || pose == null) return;

    final paint = Paint()
      ..color = Colors.cyanAccent.withOpacity(0.85)
      ..strokeWidth = 2.5
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    bool first = true;
    for (final pt in trajectory) {
      final px = cx + (pt.x - pose.x) * scaleX;
      final py = cy - (pt.y - pose.y) * scaleY;
      if (first) {
        path.moveTo(px, py);
        first = false;
      } else {
        path.lineTo(px, py);
      }
    }
    canvas.drawPath(path, paint);

    final goal = trajectory.last;
    canvas.drawCircle(
      Offset(cx + (goal.x - pose.x) * scaleX, cy - (goal.y - pose.y) * scaleY),
      5,
      Paint()..color = Colors.cyanAccent,
    );
  }

  /// Global path is already in odom frame (backend transforms via exact T_odom_map).
  void _drawGlobalPath(Canvas canvas, double cx, double cy,
      double scaleX, double scaleY, Pose? odomPose) {
    if (globalPath.length < 2 || odomPose == null) return;

    Offset toCanvas(TrajPoint pt) => Offset(
      cx + (pt.x - odomPose.x) * scaleX,
      cy - (pt.y - odomPose.y) * scaleY,
    );

    final linePaint = Paint()
      ..color = const Color(0xFF69F0AE).withOpacity(0.9)
      ..strokeWidth = 2.5
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    bool first = true;
    for (final pt in globalPath) {
      final c = toCanvas(pt);
      if (first) {
        path.moveTo(c.dx, c.dy);
        first = false;
      } else {
        path.lineTo(c.dx, c.dy);
      }
    }
    canvas.drawPath(path, linePaint);

    // Target marker at path end.
    final gc = toCanvas(globalPath.last);
    canvas.drawCircle(gc, 7, Paint()..color = const Color(0xFF69F0AE));
    canvas.drawCircle(gc, 7,
        Paint()..color = Colors.white..style = PaintingStyle.stroke..strokeWidth = 2.0);
    canvas.drawCircle(gc, 3, Paint()..color = Colors.white);
  }

  void _drawNavTarget(Canvas canvas, double cx, double cy,
      double scaleX, double scaleY, Pose odomPose, TrajPoint target) {
    final px = cx + (target.x - odomPose.x) * scaleX;
    final py = cy - (target.y - odomPose.y) * scaleY;
    final c = Offset(px, py);
    const r = 10.0;
    const arm = 6.0;
    final ring = Paint()
      ..color = const Color(0xFFFF6D00)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.5;
    final cross = Paint()
      ..color = const Color(0xFFFF6D00)
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(c, r, ring);
    canvas.drawCircle(c, 3, Paint()..color = const Color(0xFFFF6D00));
    canvas.drawLine(Offset(px - r - arm, py), Offset(px - r + arm, py), cross);
    canvas.drawLine(Offset(px + r - arm, py), Offset(px + r + arm, py), cross);
    canvas.drawLine(Offset(px, py - r - arm), Offset(px, py - r + arm), cross);
    canvas.drawLine(Offset(px, py + r - arm), Offset(px, py + r + arm), cross);
  }

  void _drawFootprint(Canvas canvas, double cx, double cy,
      double scaleX, double scaleY, Pose? pose) {
    if (footprint.isEmpty || pose == null) return;

    final pts = <Offset>[];
    for (int i = 0; i < footprint.length; i++) {
      pts.add(Offset(
        cx + (footprint[i].x - pose.x) * scaleX,
        cy - (footprint[i].y - pose.y) * scaleY,
      ));
    }

    final path = Path()..moveTo(pts[0].dx, pts[0].dy);
    for (int i = 1; i < pts.length; i++) {
      path.lineTo(pts[i].dx, pts[i].dy);
    }
    path.close();

    // 这条多边形是规划器的碰撞圆(绕控制点的扫掠圆)，不是车 —— 所以只描边、压淡。
    canvas.drawPath(path,
        Paint()
          ..color = const Color(0xFF29B6F6).withOpacity(0.55)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.6
          ..strokeJoin = StrokeJoin.round);

    // 顶点标记只对方形底盘有意义。圆盘发的是 16 边形，逐点画 4px 实心圆就糊成一圈点，
    // 比车体本身还大 —— 这正是页面上「小车变成一堆圆点」的来源。
    if (pts.length <= 8) {
      for (final p in pts) {
        canvas.drawCircle(p, 4, Paint()..color = const Color(0xFF29B6F6));
      }
    }

    final ch = chassis;
    if (ch == null || ch.bodyRadiusM <= 0) return;
    // 车体圆。圆心从多边形的形心推出来(=控制点)，再沿朝向前移 bodyOffsetX ——
    // 不去猜 odomPose 是哪个点，省得又把坐标系搞错。
    double sx = 0, sy = 0;
    for (final p in pts) { sx += p.dx; sy += p.dy; }
    final hub = Offset(sx / pts.length, sy / pts.length);
    final yaw = pose.yaw;
    final body = Offset(hub.dx + math.cos(yaw) * ch.bodyOffsetXM * scaleX,
                        hub.dy - math.sin(yaw) * ch.bodyOffsetXM * scaleY);
    final rBody = ch.bodyRadiusM * scaleX;
    canvas.drawCircle(body, rBody,
        Paint()..color = const Color(0xFF64B5F6).withOpacity(0.28));
    canvas.drawCircle(body, rBody,
        Paint()
          ..color = const Color(0xFF29B6F6)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2.4);
  }

  void _drawRobotArrow(Canvas canvas, Offset center, double yaw) {
    final cosY = math.cos(yaw);
    final sinY = math.sin(yaw);

    final tip   = Offset(center.dx + cosY * 8, center.dy - sinY * 8);
    final left  = Offset(center.dx - sinY * 3.5, center.dy - cosY * 3.5);
    final right = Offset(center.dx + sinY * 3.5, center.dy + cosY * 3.5);
    final base  = Offset(center.dx - cosY * 3, center.dy + sinY * 3);

    final path = Path()
      ..moveTo(tip.dx, tip.dy)
      ..lineTo(left.dx, left.dy)
      ..lineTo(base.dx, base.dy)
      ..lineTo(right.dx, right.dy)
      ..close();

    canvas.drawPath(path, Paint()..color = Colors.white);
    canvas.drawPath(path,
        Paint()..color = Colors.black45..style = PaintingStyle.stroke..strokeWidth = 0.8);
  }

  @override
  bool shouldRepaint(LocalPlanningPainter old) =>
      trajectory != old.trajectory ||
      globalPath != old.globalPath ||
      footprint != old.footprint ||
      gridInfo != old.gridInfo ||
      odomPose != old.odomPose ||
      showTrajectory != old.showTrajectory ||
      showGlobalPath != old.showGlobalPath ||
      showFootprint != old.showFootprint ||
      navTargetPose != old.navTargetPose;
}
