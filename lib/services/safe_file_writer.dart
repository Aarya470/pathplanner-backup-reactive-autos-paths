import 'package:file/file.dart';
import 'package:pathplanner/services/log.dart';

/// Saves files one write at a time per file, so quick repeated saves (like dragging a path)
/// never overlap. Windows refuses a write while the same file is still being written, which
/// used to show up as an uncaught FileSystemException.
///
/// If a write is already running for a file, only the newest content is kept and written
/// when it finishes. A failed write is retried a few times before it is logged.
class SafeFileWriter {
  static const int _maxAttempts = 4;

  static final Map<String, String> _queued = {};
  static final Map<String, Future<bool>> _running = {};

  /// Write [content] to [file]. The write starts right away unless another write to the same
  /// file is still running.
  ///
  /// Returns true once the content (or newer content queued after it) is on disk.
  static Future<bool> write(File file, String content) {
    final key = file.path;
    final running = _running[key];
    if (running != null) {
      _queued[key] = content;
      return running.then((_) => _running[key] ?? Future.value(true));
    }
    final future = _write(file, key, content);
    _running[key] = future;
    return future;
  }

  static Future<bool> _write(File file, String key, String content) async {
    bool ok = false;
    for (int attempt = 1; attempt <= _maxAttempts; attempt++) {
      try {
        await file.writeAsString(content);
        ok = true;
        break;
      } catch (ex, stack) {
        if (attempt == _maxAttempts) {
          Log.error('Could not save ${file.basename}', ex, stack);
        } else {
          await Future.delayed(Duration(milliseconds: 150 * attempt));
        }
      }
    }

    _running.remove(key);
    final next = _queued.remove(key);
    if (next != null) {
      final future = _write(file, key, next);
      _running[key] = future;
      return future;
    }
    return ok;
  }
}
