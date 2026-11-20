import 'dart:io';

import 'package:http/http.dart' as http;
import 'package:path/path.dart';

/// Installs app updates on Windows: downloads the new installer, runs it silently, and closes
/// the app. The installer reopens the app when it finishes.
class AppUpdater {
  /// True on Windows when this copy was installed with the installer (not the portable zip).
  static bool get canInstallUpdates {
    if (!Platform.isWindows) return false;
    final appDir = File(Platform.resolvedExecutable).parent.path;
    return File(join(appDir, 'unins000.exe')).existsSync();
  }

  /// Download the installer at [url] to the temp folder.
  ///
  /// [onProgress] gets a value from 0 to 1 when the download size is known.
  static Future<File> downloadInstaller(
      String url, void Function(double progress) onProgress) async {
    final client = http.Client();
    try {
      final response = await client.send(http.Request('GET', Uri.parse(url)));
      if (response.statusCode != 200) {
        throw HttpException('Download failed (HTTP ${response.statusCode})');
      }
      final total = response.contentLength ?? 0;
      final file = File(join(Directory.systemTemp.path,
          'PathPlannerBackupPaths-update-setup.exe'));
      final sink = file.openWrite();
      int received = 0;
      await for (final chunk in response.stream) {
        sink.add(chunk);
        received += chunk.length;
        if (total > 0) onProgress(received / total);
      }
      await sink.close();
      return file;
    } finally {
      client.close();
    }
  }

  /// Start the installer silently and close the app so it can be replaced.
  static Future<void> installAndExit(File installer) async {
    await Process.start(
      installer.path,
      ['/SILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/CLOSEAPPLICATIONS'],
      mode: ProcessStartMode.detached,
    );
    exit(0);
  }
}
