import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'core/providers.dart';
import 'pages/home_page.dart';
import 'pages/setup_page.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final prefs = await SharedPreferences.getInstance();
  // The backend serves this bundle itself, so the page's own origin already IS the
  // device. Derive it rather than hardcoding: the USB address stopped resolving the
  // moment the C port was switched to USB host for the WiFi dongle, and a hardcoded
  // default then reports "device offline" while the page is being served by the very
  // board it says is unreachable.
  final defaultDeviceIp =
      kIsWeb && Uri.base.host.isNotEmpty ? Uri.base.host : '169.254.10.1';
  final savedIp = prefs.getString('device_ip') ?? defaultDeviceIp;

  runApp(
    ProviderScope(
      overrides: [
        sharedPreferencesProvider.overrideWithValue(prefs),
        deviceIpProvider.overrideWith((ref) => savedIp),
      ],
      child: const TinyNavApp(),
    ),
  );
}

class TinyNavApp extends ConsumerWidget {
  const TinyNavApp({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final ip = ref.watch(deviceIpProvider);
    return MaterialApp(
      title: 'TinyNav',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF38D3FF),
          primary: const Color(0xFF38D3FF),
          brightness: Brightness.dark,
        ),
        useMaterial3: true,
        fontFamily: 'RobotoLocal',
        scaffoldBackgroundColor: const Color(0xFF0B1118),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFF0F1822),
          foregroundColor: Color(0xFFE6EEF7),
          elevation: 0,
          surfaceTintColor: Colors.transparent,
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: FilledButton.styleFrom(
            backgroundColor: const Color(0xFF38D3FF),
            foregroundColor: const Color(0xFF04131B),
            shape: const StadiumBorder(),
          ),
        ),
        outlinedButtonTheme: OutlinedButtonThemeData(
          style: OutlinedButton.styleFrom(
            foregroundColor: const Color(0xFFB8C8D9),
            side: const BorderSide(color: Color(0xFF2D3E50)),
            shape: const StadiumBorder(),
          ),
        ),
        cardTheme: CardThemeData(
          elevation: 0,
          margin: EdgeInsets.zero,
          color: const Color(0xFF111A24),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
        ),
      ),
      // No 1:1 landscape lock. Upstream clamps web content to a square when the
      // viewport is wider than tall, which suits a tablet held sideways but not the
      // laptop browser this is actually driven from: on a 16:9 window the whole app
      // collapses into a centre square, the camera panel becomes that square's top
      // 2/7 -- roughly 3.5:1 -- and BoxFit.cover then crops about two thirds off a
      // 640x544 frame. Let the layout use the window it was given.
      // Switches automatically when deviceIpProvider changes.
      home: ip == null ? const SetupPage() : const HomePage(),
    );
  }
}
