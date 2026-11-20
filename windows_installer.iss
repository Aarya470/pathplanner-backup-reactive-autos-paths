#define MyAppName "PathPlanner Backup Paths"
#define MyAppFolderName "PathPlannerBackupPaths"
#define MyAppPublisher "Aaryan Roy (FRC Team 1089)"
#define MyAppURL "https://github.com/Aarya470/pathplanner-backup-reactive-autos-paths/"
#define MyAppExeName "pathplanner.exe"
#define AppVersionName GetStringFileInfo('build\windows\x64\runner\Release\pathplanner.exe', 'ProductVersion')
#define ApplicationVersion GetVersionNumbersString('build\windows\x64\runner\Release\pathplanner.exe')

[Setup]
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{d0cedce3-23c4-4a56-9b2d-5a3aec40c830}
AppName={#MyAppName}
AppVersion={#ApplicationVersion}
AppVerName={#MyAppName} {#AppVersionName}
VersionInfoVersion={#ApplicationVersion}
VersionInfoCompany={#MyAppPublisher}
VersionInfoCopyright=Copyright (c) 2026 Aaryan Roy, FRC Team 1089. Based on PathPlanner (c) 2022 Michael Jansen.
AppCopyright=Copyright (c) 2026 Aaryan Roy, FRC Team 1089
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf}\{#MyAppFolderName}
; "ArchitecturesAllowed=x64compatible" specifies that Setup cannot run
; on anything but x64 and Windows 11 on Arm.
ArchitecturesAllowed=x64compatible
; "ArchitecturesInstallIn64BitMode=x64compatible" requests that the
; install be done in "64-bit mode" on x64 or Windows 11 on Arm,
; meaning it should use the native 64-bit Program Files directory and
; the 64-bit view of the registry.
ArchitecturesInstallIn64BitMode=x64compatible
DisableProgramGroupPage=yes
LicenseFile=TERMS.txt
PrivilegesRequired=lowest
CloseApplications=force
PrivilegesRequiredOverridesAllowed=dialog
OutputDir=build\windows\x64\installer
OutputBaseFilename=pathplanner-backup-reactive-autos-paths-setup
SetupIconFile=windows\icon.ico
Compression=lzma
SolidCompression=yes
WizardStyle=modern
DisableWelcomePage=no

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Messages]
WelcomeLabel2=This will install [name/ver] on your computer.%n%nMade by Aaryan Roy on FRC Team 1089 (Mercury).%n%nOn the next page, please read and accept the terms of use.%n%nIt is recommended that you close all other applications before continuing.
LicenseLabel3=Please read the following terms of use. You must accept them before continuing with the installation.

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "build\windows\x64\runner\Release\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\windows\x64\runner\Release\*.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\windows\x64\runner\Release\data\*"; DestDir: "{app}\data"; Flags: ignoreversion recursesubdirs createallsubdirs
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
Filename: "{app}\{#MyAppExeName}"; Flags: nowait; Check: WizardSilent