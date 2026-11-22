import 'dart:convert';

import 'package:file/file.dart';
import 'package:github/github.dart';
import 'package:path/path.dart';
import 'package:pathplanner/services/log.dart';
import 'package:version/version.dart';
import 'package:http/http.dart' as http;

class UpdateChecker {
  final GitHub _github;
  static const String _jsonURL =
      'https://aarya470.github.io/pathplanner-backup-reactive-autos-paths/ReactiveAutonomousPathing.json';

  UpdateChecker() : _github = GitHub();

  Future<bool> isGuiUpdateAvailable(String currentVersion) async {
    try {
      Release latestRelease = await _github.repositories.getLatestRelease(
          RepositorySlug(
              'Aarya470', 'pathplanner-backup-reactive-autos-paths'));
      String latestVersion = latestRelease.tagName!.substring(1);

      Log.verbose(
          'Current App Version: $currentVersion, Latest Release: $latestVersion');

      Version current = Version.parse(currentVersion);
      Version latest = Version.parse(latestVersion);

      if (current.major == 0 && current.minor == 0 && current.patch == 0) {
        // Dev build
        return false;
      }

      return latest > current;
    } catch (_) {
      return false;
    }
  }

  /// Download link for the Windows installer in the latest release, or null if there isn't one.
  Future<String?> getWindowsInstallerUrl() async {
    try {
      Release latestRelease = await _github.repositories.getLatestRelease(
          RepositorySlug(
              'Aarya470', 'pathplanner-backup-reactive-autos-paths'));
      for (ReleaseAsset asset in latestRelease.assets ?? []) {
        if ((asset.name ?? '').endsWith('-Windows-setup.exe')) {
          return asset.browserDownloadUrl;
        }
      }
    } catch (e) {
      Log.error('Could not find the update installer', e);
    }
    return null;
  }

  /// Checks the robot project's robot library.
  ///
  /// Returns [RobotLibraryUpdate.oldLibrary] if the project still uses the 1.x library
  /// (PathplannerLib-BackupPaths), which 2.0 replaced, [RobotLibraryUpdate.newerVersion] if its
  /// Reactive Autonomous Pathing is older than the latest release, and otherwise
  /// [RobotLibraryUpdate.none].
  Future<RobotLibraryUpdate> checkRobotLibrary(
      {required Directory projectDir, required FileSystem fs}) async {
    File oldLibraryFile = fs.file(
        join(projectDir.path, 'vendordeps', 'PathplannerLib-BackupPaths.json'));
    if (await oldLibraryFile.exists()) {
      return RobotLibraryUpdate.oldLibrary;
    }

    File vendorDepFile = fs.file(
        join(projectDir.path, 'vendordeps', 'ReactiveAutonomousPathing.json'));

    if (await vendorDepFile.exists()) {
      String fileContent = await vendorDepFile.readAsString();
      Map<String, dynamic> localJson = jsonDecode(fileContent);

      try {
        String remote = await http.read(Uri.parse(_jsonURL));
        Map<String, dynamic> remoteJson = jsonDecode(remote);

        String localVersion = localJson['version'];
        String remoteVersion = remoteJson['version'];

        Log.verbose(
            'Current Reactive Autonomous Pathing Version: $localVersion, Latest Release: $remoteVersion');

        if (Version.parse(remoteVersion) > Version.parse(localVersion)) {
          return RobotLibraryUpdate.newerVersion;
        }
      } catch (e) {
        // Can't get file. Probably not connected to internet
      }
    }

    return RobotLibraryUpdate.none;
  }
}

/// What the robot project's robot library needs, from [UpdateChecker.checkRobotLibrary].
enum RobotLibraryUpdate {
  /// Nothing: it is up to date, or the project doesn't use it
  none,

  /// A newer Reactive Autonomous Pathing is out
  newerVersion,

  /// The project still uses the 1.x library, PathplannerLib-BackupPaths
  oldLibrary,
}
