import 'package:flutter/material.dart';
import 'package:pathplanner/services/app_updater.dart';
import 'package:pathplanner/services/update_checker.dart';
import 'package:url_launcher/url_launcher.dart';

class UpdateCard extends StatefulWidget {
  final String currentVersion;
  final UpdateChecker updateChecker;

  const UpdateCard({
    required this.currentVersion,
    required this.updateChecker,
    super.key,
  });

  @override
  State<UpdateCard> createState() => _UpdateCardState();
}

class _UpdateCardState extends State<UpdateCard> with TickerProviderStateMixin {
  late AnimationController _updateController;
  late Animation<Offset> _offsetAnimation;
  bool _visibile = false;
  final String _releaseURL =
      'https://github.com/Aarya470/pathplanner-backup-reactive-autos-paths/releases/latest';

  @override
  void initState() {
    super.initState();
    _updateController = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 400));
    _offsetAnimation =
        Tween<Offset>(begin: const Offset(0, -0.05), end: Offset.zero)
            .animate(CurvedAnimation(
      parent: _updateController,
      curve: Curves.ease,
    ));

    widget.updateChecker
        .isGuiUpdateAvailable(widget.currentVersion)
        .then((value) {
      if (value) {
        setState(() {
          _visibile = value;
          _updateController.forward();
        });
      }
    });
  }

  Future<void> _openReleasePage() async {
    Uri url = Uri.parse(_releaseURL);
    if (await canLaunchUrl(url)) {
      launchUrl(url);
    }
  }

  @override
  void dispose() {
    _updateController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Visibility(
      visible: _visibile,
      child: SlideTransition(
        position: _offsetAnimation,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(8.0, 4.0, 8.0, 0),
          child: Card(
            child: Padding(
              padding: const EdgeInsets.all(10.0),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    'PathPlanner update available!',
                    style:
                        TextStyle(fontSize: 18, color: colorScheme.onSurface),
                  ),
                  const SizedBox(width: 12),
                  ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: colorScheme.primaryContainer,
                      foregroundColor: colorScheme.onPrimaryContainer,
                    ),
                    onPressed: () async {
                      if (AppUpdater.canInstallUpdates) {
                        showDialog(
                          context: context,
                          barrierDismissible: false,
                          builder: (context) => _InstallUpdateDialog(
                            updateChecker: widget.updateChecker,
                            onFallback: _openReleasePage,
                          ),
                        );
                      } else {
                        _openReleasePage();
                      }
                    },
                    child: const Text('Update'),
                  ),
                  const SizedBox(width: 8),
                  ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: colorScheme.surfaceContainerHighest,
                      foregroundColor: colorScheme.onSurfaceVariant,
                    ),
                    onPressed: () {
                      setState(() {
                        _visibile = false;
                      });
                    },
                    child: const Text('Dismiss'),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// Downloads and installs the update on Windows, showing progress.
class _InstallUpdateDialog extends StatefulWidget {
  final UpdateChecker updateChecker;
  final VoidCallback onFallback;

  const _InstallUpdateDialog({
    required this.updateChecker,
    required this.onFallback,
  });

  @override
  State<_InstallUpdateDialog> createState() => _InstallUpdateDialogState();
}

class _InstallUpdateDialogState extends State<_InstallUpdateDialog> {
  double? _progress;
  String _status = 'Finding the update...';
  bool _failed = false;

  @override
  void initState() {
    super.initState();
    _install();
  }

  Future<void> _install() async {
    try {
      final url = await widget.updateChecker.getWindowsInstallerUrl();
      if (url == null) {
        throw Exception('The latest release has no Windows installer');
      }
      if (!mounted) return;
      setState(() => _status = 'Downloading the update...');
      final installer = await AppUpdater.downloadInstaller(url, (progress) {
        if (mounted) setState(() => _progress = progress);
      });
      if (!mounted) return;
      setState(() {
        _progress = null;
        _status = 'Installing. PathPlanner Backup Paths will close and reopen.';
      });
      await Future.delayed(const Duration(seconds: 1));
      await AppUpdater.installAndExit(installer);
    } catch (e) {
      if (mounted) {
        setState(() {
          _failed = true;
          _status =
              'The update could not be installed automatically. You can download it from the release page instead.';
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return AlertDialog(
      backgroundColor: colorScheme.surface,
      surfaceTintColor: colorScheme.surfaceTint,
      title: const Text('Updating'),
      content: SizedBox(
        width: 400,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(_status),
            if (!_failed) ...[
              const SizedBox(height: 16),
              LinearProgressIndicator(value: _progress),
            ],
          ],
        ),
      ),
      actions: [
        if (_failed) ...[
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text('Close'),
          ),
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              widget.onFallback();
            },
            child: const Text('Open release page'),
          ),
        ],
      ],
    );
  }
}
