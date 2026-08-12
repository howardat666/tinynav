import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../core/models.dart';
import '../core/providers.dart';

const _kBg = Color(0xFF0F1822);
const _kBorder = Color(0xFF223243);
const _kLabel = Color(0xFF7F94A8);
const _kValue = Color(0xFFD6E3F0);
const _kAccent = Color(0xFF38D3FF);
const _kWarn = Color(0xFFFFB347);
const _kBad = Color(0xFFFF6B6B);

/// Why relocalisation stopped, in the operator's words rather than the code's.
const _kReasonText = <String, String>{
  'few_landmarks': '当前画面和地图关键帧匹配上的特征点太少',
  'pnp_inliers': '匹配上了但解算位姿时内点不够',
  'solve': '位姿解算本身失败',
  'outside_map': '解出的位姿落在地图范围外，已丢弃',
  'no_candidates': '场景检索没找到相似的地图关键帧',
  'no_intrinsics': '还没收到相机内参',
  'unknown': '未分类',
};

/// Collapsible diagnostics strip between the map and the joystick.
///
/// Everything here already existed in /device/status and the planning stream and had
/// no UI consumer at all, so a failing run could only be read off the board's logs.
class DebugPanel extends ConsumerStatefulWidget {
  const DebugPanel({super.key});

  @override
  ConsumerState<DebugPanel> createState() => _DebugPanelState();
}

class _DebugPanelState extends ConsumerState<DebugPanel> {
  bool _open = false;

  @override
  Widget build(BuildContext context) {
    final status = ref.watch(deviceStatusProvider).valueOrNull;
    final planning = ref.watch(planningStreamProvider).valueOrNull;
    final reloc = status?.reloc;

    return DecoratedBox(
      decoration: const BoxDecoration(
        color: _kBg,
        border: Border(top: BorderSide(color: _kBorder)),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          _header(reloc),
          if (_open)
            ConstrainedBox(
              // Bounded so expanding never squeezes the joystick off screen; the
              // content scrolls instead.
              constraints: const BoxConstraints(maxHeight: 190),
              child: SingleChildScrollView(
                padding: const EdgeInsets.fromLTRB(12, 0, 12, 10),
                child: _body(reloc, planning, status),
              ),
            ),
        ],
      ),
    );
  }

  Widget _header(RelocStats? reloc) {
    return InkWell(
      onTap: () => setState(() => _open = !_open),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
        child: Row(
          children: [
            Icon(_open ? Icons.expand_more : Icons.chevron_right,
                size: 18, color: _kLabel),
            const SizedBox(width: 4),
            const Text('详细信息',
                style: TextStyle(
                    color: _kValue, fontSize: 12.5, fontWeight: FontWeight.w600)),
            const Spacer(),
            if (!_open) _summary(reloc),
          ],
        ),
      ),
    );
  }

  /// The one line worth seeing without expanding: how stale the map pose is.
  Widget _summary(RelocStats? reloc) {
    final age = reloc?.secondsSinceLastSuccess;
    if (age == null) {
      return const Text('重定位 未成功过',
          style: TextStyle(color: _kLabel, fontSize: 11.5));
    }
    final color = age < 5 ? _kAccent : (age < 20 ? _kWarn : _kBad);
    final rate = reloc?.window?.successRate;
    return Text(
      '重定位 ${_age(age)}'
      '${rate == null ? '' : ' · ${(rate * 100).round()}%'}',
      style: TextStyle(color: color, fontSize: 11.5),
    );
  }

  Widget _body(RelocStats? reloc, PlanningState? planning, DeviceStatus? status) {
    final w = reloc?.window;
    final t = reloc?.total;
    final code = reloc?.lastFailureCode;
    final age = reloc?.secondsSinceLastSuccess;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _section('重定位'),
        _row('上次成功', age == null ? '从未成功' : _age(age),
            color: age == null ? _kBad : (age < 5 ? _kValue : _kWarn)),
        if (code != null) ...[
          _row('最近失败', '$code — ${_kReasonText[code] ?? ''}', color: _kWarn),
          if (reloc?.lastFailureReason != null)
            _row('', reloc!.lastFailureReason!, small: true),
        ],
        if (w != null)
          _row('最近 ${w.spanS.round()} 秒',
              '${w.keyframes} 关键帧 → ${w.attempts} 次尝试 → ${w.success} 次成功'
              '${w.successRate == null ? '' : ' (${(w.successRate! * 100).round()}%)'}'),
        if (w != null && w.byCode.isNotEmpty)
          _row('失败分布',
              w.byCode.entries.map((e) => '${e.key}=${e.value}').join('  ')),
        if (w != null && (w.droppedStale > 0 || w.skippedRateLimit > 0))
          _row('未尝试', '过期丢弃 ${w.droppedStale}  ·  限流跳过 ${w.skippedRateLimit}'),
        if (t != null)
          _row('累计', '${t.success}/${t.attempts}'
              '${t.successRate == null ? '' : ' (${(t.successRate! * 100).round()}%)'}'),
        const SizedBox(height: 8),
        _section('位姿'),
        _row('地图系', _pose(planning?.mapPose),
            color: planning?.mapPose == null ? _kLabel : _kValue),
        _row('里程计', _pose(planning?.odomPose)),
        const SizedBox(height: 8),
        _section('状态'),
        _row('导航', '${status?.navStatus ?? '-'}'
            '  (raw ${status?.rawState ?? '-'})'
            '${(status?.navPaused ?? false) ? '  · 已暂停' : ''}',
            color: (status?.navPaused ?? false) ? _kWarn : _kValue),
        _row('导航节点', (status?.navNodesRunning ?? false) ? '运行中' : '未启动',
            color: (status?.navNodesRunning ?? false) ? _kValue : _kLabel),
        _row('已定位', (planning?.localized ?? false) ? '是' : '否',
            color: (planning?.localized ?? false) ? _kValue : _kBad),
      ],
    );
  }

  Widget _section(String s) => Padding(
        padding: const EdgeInsets.only(top: 2, bottom: 3),
        child: Text(s,
            style: const TextStyle(
                color: _kAccent,
                fontSize: 11,
                fontWeight: FontWeight.w700,
                letterSpacing: 0.6)),
      );

  Widget _row(String label, String value, {Color? color, bool small = false}) => Padding(
        padding: const EdgeInsets.only(bottom: 2),
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            SizedBox(
              width: 72,
              child: Text(label,
                  style: const TextStyle(color: _kLabel, fontSize: 11.5)),
            ),
            Expanded(
              child: Text(value,
                  style: TextStyle(
                      color: color ?? _kValue,
                      fontSize: small ? 10.5 : 11.5,
                      fontFeatures: const [FontFeature.tabularFigures()])),
            ),
          ],
        ),
      );

  static String _age(double s) =>
      s < 60 ? '${s.toStringAsFixed(1)} 秒前' : '${(s / 60).toStringAsFixed(1)} 分钟前';

  static String _pose(Pose? p) => p == null
      ? '无'
      : 'x=${p.x.toStringAsFixed(2)}  y=${p.y.toStringAsFixed(2)}  '
          'yaw=${(p.yaw * 180 / 3.14159265).toStringAsFixed(0)}°';
}
