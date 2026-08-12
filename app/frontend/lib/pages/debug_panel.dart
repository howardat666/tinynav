import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../core/models.dart';
import '../core/providers.dart';

// English throughout, like the rest of the app. CanvasKit renders with the fonts the
// bundle ships and does no system fallback, and the bundled face is Roboto, so any CJK
// glyph draws as a box. Chinese here would mean subsetting a CJK font into assets and
// re-subsetting it every time a string changes.
const _kBg = Color(0xFF0D151E);
const _kLabel = Color(0xFF7F94A8);
const _kValue = Color(0xFFD6E3F0);
const _kAccent = Color(0xFF38D3FF);
const _kWarn = Color(0xFFFFB347);
const _kBad = Color(0xFFFF6B6B);

/// Why relocalisation stopped, spelled out. The codes are map_node's slugs.
const _kReasonText = <String, String>{
  'few_landmarks': 'too few feature matches against the map keyframe',
  'pnp_inliers': 'matched, but too few inliers to solve the pose',
  'solve': 'pose solver itself failed',
  'outside_map': 'solved pose landed outside the map, discarded',
  'no_candidates': 'no similar map keyframe retrieved',
  'no_intrinsics': 'camera intrinsics not received yet',
  'unknown': 'unclassified',
};

/// The diagnostics page that replaces the joystick when toggled.
///
/// Every number here already existed on a topic or in /device/status and had no UI
/// consumer, so answering "is it localised, and if not why" meant reading the board's
/// logs over ssh.
class DebugPanel extends ConsumerWidget {
  const DebugPanel({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final status = ref.watch(deviceStatusProvider).valueOrNull;
    final planning = ref.watch(planningStreamProvider).valueOrNull;
    final reloc = status?.reloc;
    final diag = planning?.diag;

    final columns = <Widget>[
      _column('RELOCALIZATION', _relocRows(reloc)),
      _column('PERCEPTION', _perceptionRows(reloc, diag)),
      _column('POSE', _poseRows(planning)),
    ];

    return Container(
      color: _kBg,
      padding: const EdgeInsets.fromLTRB(12, 8, 12, 8),
      child: LayoutBuilder(
        builder: (context, c) {
          // Three abreast on a laptop window, two on a tablet, stacked on a phone.
          final perRow = c.maxWidth > 720 ? 3 : (c.maxWidth > 460 ? 2 : 1);
          final rows = <Widget>[];
          for (var i = 0; i < columns.length; i += perRow) {
            rows.add(Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                for (var j = i; j < i + perRow && j < columns.length; j++) ...[
                  Expanded(child: columns[j]),
                  if (j < i + perRow - 1 && j < columns.length - 1)
                    const SizedBox(width: 14),
                ],
              ],
            ));
          }
          return SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                for (var i = 0; i < rows.length; i++) ...[
                  rows[i],
                  if (i < rows.length - 1) const SizedBox(height: 8),
                ],
              ],
            ),
          );
        },
      ),
    );
  }

  List<Widget> _relocRows(RelocStats? r) {
    if (r == null) {
      return const [_Row('', 'no data -- enable nav nodes', color: _kLabel)];
    }
    final age = r.secondsSinceLastSuccess;
    final w = r.window;
    final t = r.total;
    return [
      _Row('last ok', age == null ? 'never' : _age(age),
          color: age == null ? _kBad : (age < 5 ? _kValue : (age < 20 ? _kWarn : _kBad))),
      if (w != null)
        _Row('rate', '${w.attemptHz.toStringAsFixed(2)} Hz try'
            '  ->  ${w.successHz.toStringAsFixed(2)} Hz ok'
            '${w.successRate == null ? '' : '  (${(w.successRate! * 100).round()}%)'}'),
      if (w != null)
        _Row('window', '${w.keyframes} kf / ${w.attempts} try / ${w.success} ok'
            '  over ${w.spanS.round()}s'),
      if (w != null && w.byCode.isNotEmpty)
        _Row('failures', w.byCode.entries.map((e) => '${e.key} ${e.value}').join('   '),
            color: _kWarn),
      if (w != null && (w.droppedStale > 0 || w.skippedRateLimit > 0))
        _Row('not run', 'stale ${w.droppedStale}   rate-limited ${w.skippedRateLimit}'),
      if (r.lastFailureCode != null)
        _Row('why', '${r.lastFailureCode} -- '
            '${_kReasonText[r.lastFailureCode] ?? ''}', color: _kWarn),
      if (r.lastFailureReason != null) _Row('', r.lastFailureReason!, small: true),
      if (t != null)
        _Row('lifetime', '${t.success}/${t.attempts}'
            '${t.successRate == null ? '' : '  (${(t.successRate! * 100).round()}%)'}'),
    ];
  }

  List<Widget> _perceptionRows(RelocStats? r, PlanningDiag? d) {
    final kfHz = r?.window?.keyframeHz;
    if (d == null) {
      return [
        if (kfHz != null) _Row('keyframes', '${kfHz.toStringAsFixed(2)} Hz'),
        const _Row('', 'planning is not publishing', color: _kLabel),
      ];
    }
    final fc = d.frontClearanceM;
    return [
      // "nothing within the probe" is not a distance; see _clearance_along.
      _Row('front obst',
          fc == null
              ? 'clear (> ${d.frontProbeMaxM.toStringAsFixed(2)} m)'
              : '${fc.toStringAsFixed(2)} m',
          color: d.frontBlocked ? _kBad : (fc != null && fc < 0.5 ? _kWarn : _kValue)),
      _Row('blocked at', '${d.frontBlockedAtM.toStringAsFixed(2)} m'
          '${d.frontBlocked ? '   BLOCKED' : ''}',
          color: d.frontBlocked ? _kBad : _kLabel),
      _Row('clearance', d.esdfAtRobotM == null
          ? 'off grid'
          : '${d.esdfAtRobotM!.toStringAsFixed(2)} m at robot',
          color: (d.esdfAtRobotM ?? 1) < 0.1 ? _kBad : _kValue),
      _Row('obstacles', '${d.obstacleCells} cells'),
      if (kfHz != null) _Row('keyframes', '${kfHz.toStringAsFixed(2)} Hz'),
      if (d.cycleS != null)
        _Row('planning', '${(1 / d.cycleS!).toStringAsFixed(1)} Hz'
            '  (${(d.cycleS! * 1000).round()} ms)',
            color: d.cycleS! > 0.4 ? _kWarn : _kValue),
      if (d.stampLagS != null)
        _Row('sensor lag', '${d.stampLagS!.toStringAsFixed(2)} s',
            color: d.stampLagS! > 0.5 ? _kWarn : _kValue),
    ];
  }

  List<Widget> _poseRows(PlanningState? p) => [
        _Row('map', _pose(p?.mapPose), color: p?.mapPose == null ? _kLabel : _kValue),
        _Row('odom', _pose(p?.odomPose)),
      ];

  static Widget _column(String title, List<Widget> rows) => Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: [
          Text(title,
              style: const TextStyle(
                  color: _kAccent,
                  fontSize: 12,
                  fontWeight: FontWeight.w700,
                  letterSpacing: 1.1)),
          const SizedBox(height: 5),
          ...rows,
        ],
      );

  static String _age(double s) =>
      s < 60 ? '${s.toStringAsFixed(1)} s ago' : '${(s / 60).toStringAsFixed(1)} min ago';

  static String _pose(Pose? p) => p == null
      ? '--'
      : '${p.x.toStringAsFixed(2)}, ${p.y.toStringAsFixed(2)}   '
          '${(p.yaw * 180 / 3.14159265).toStringAsFixed(0)} deg';
}

class _Row extends StatelessWidget {
  final String label;
  final String value;
  final Color? color;
  final bool small;

  const _Row(this.label, this.value, {this.color, this.small = false});

  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.only(bottom: 3),
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            SizedBox(
              width: 84,
              child: Text(label,
                  style: const TextStyle(color: _kLabel, fontSize: 13)),
            ),
            Expanded(
              child: Text(value,
                  style: TextStyle(
                      color: color ?? _kValue,
                      fontSize: small ? 11.5 : 13,
                      height: 1.25,
                      fontFeatures: const [FontFeature.tabularFigures()])),
            ),
          ],
        ),
      );
}
