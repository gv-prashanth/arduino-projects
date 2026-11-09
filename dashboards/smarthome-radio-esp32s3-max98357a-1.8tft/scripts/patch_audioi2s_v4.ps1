# Apply the same reviewed manifest as patch_audioi2s_v4.py; no Python required.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$LibraryPath,
    [switch]$Check
)
$ErrorActionPreference = 'Stop'
$utf8 = New-Object System.Text.UTF8Encoding($false, $true)

function Get-BytesHash([byte[]]$Bytes) {
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try { return ([BitConverter]::ToString($sha.ComputeHash($Bytes))).Replace('-', '').ToLowerInvariant() }
    finally { $sha.Dispose() }
}

try {
    $specRoot = Join-Path $PSScriptRoot 'audioi2s_v4'
    $spec = [IO.File]::ReadAllText((Join-Path $specRoot 'patch.json'), $utf8) | ConvertFrom-Json
    if ($spec.schema_version -ne 1) { throw 'Unsupported patch manifest' }
    $root = (Resolve-Path -LiteralPath $LibraryPath).Path
    if ((Split-Path $root -Leaf) -eq 'src') { $root = Split-Path $root -Parent }
    $properties = [IO.File]::ReadAllText((Join-Path $root 'library.properties'), $utf8)
    $versions = [regex]::Matches($properties, '(?m)^version\s*=\s*([^\r\n]+)')
    if ($versions.Count -ne 1 -or $versions[0].Groups[1].Value.Trim() -ne $spec.library_version) {
        throw 'Expected ESP32-audioI2S version 4.0.0'
    }
    $helperBytes = [IO.File]::ReadAllBytes((Join-Path $specRoot $spec.helper.source))
    if ((Get-BytesHash $helperBytes) -ne $spec.helper.sha256) { throw 'Patch helper checksum mismatch' }
    $helperPath = Join-Path $root $spec.helper.destination
    $entries = @()
    foreach ($property in $spec.files.PSObject.Properties) {
        $file = $property.Value
        $path = Join-Path $root $file.path
        $bytes = [IO.File]::ReadAllBytes($path)
        $entries += [PSCustomObject]@{
            Name = $property.Name; Spec = $file; Path = $path; Original = $bytes
            Hash = (Get-BytesHash $bytes); Backup = ($path + $spec.backup_suffix); Patched = $null
        }
    }
    $pristine = @($entries | Where-Object { $_.Hash -ne $_.Spec.original_sha256 }).Count -eq 0
    $patched = @($entries | Where-Object { $_.Hash -ne $_.Spec.patched_sha256 }).Count -eq 0
    if ($patched) {
        foreach ($entry in $entries) {
            if ((Get-BytesHash ([IO.File]::ReadAllBytes($entry.Backup))) -ne $entry.Spec.original_sha256) {
                throw ('Original backup mismatch: ' + $entry.Name)
            }
        }
        if ((Get-BytesHash ([IO.File]::ReadAllBytes($helperPath))) -ne $spec.helper.sha256) {
            throw 'Installed helper mismatch'
        }
        Write-Output 'ESP32-audioI2S 4.0.0 reserve patch: already patched'
        exit 0
    }
    if (-not $pristine) { throw 'Unknown or partial source modification; refusing to patch' }
    if (Test-Path -LiteralPath $helperPath) { throw 'Helper already exists beside pristine sources; refusing a partial installation' }
    foreach ($entry in $entries) {
        if (Test-Path -LiteralPath $entry.Backup) { throw ('Backup already exists: ' + $entry.Backup) }
        $content = $utf8.GetString($entry.Original)
        foreach ($replacement in $entry.Spec.replacements) {
            if ([regex]::Matches($content, [regex]::Escape($replacement.old)).Count -ne 1) {
                throw ('Missing or ambiguous patch insertion point: ' + $entry.Name)
            }
            $content = $content.Replace($replacement.old, $replacement.new)
        }
        $entry.Patched = $utf8.GetBytes($content)
        if ((Get-BytesHash $entry.Patched) -ne $entry.Spec.patched_sha256) {
            throw ('Patched checksum mismatch: ' + $entry.Name)
        }
    }
    if ($Check) {
        Write-Output 'ESP32-audioI2S 4.0.0 reserve patch: pristine (check only)'
        exit 0
    }
    # Verify everything before changing anything. Keep exact originals beside
    # both source files, and roll back all writes if installation fails.
    $backedUp = @()
    $helperWritten = $false
    try {
        foreach ($entry in $entries) {
            $backedUp += $entry
            [IO.File]::WriteAllBytes($entry.Backup, $entry.Original)
        }
        $helperWritten = $true
        [IO.File]::WriteAllBytes($helperPath, $helperBytes)
        foreach ($entry in $entries) { [IO.File]::WriteAllBytes($entry.Path, $entry.Patched) }
    } catch {
        foreach ($entry in $backedUp) {
            [IO.File]::WriteAllBytes($entry.Path, $entry.Original)
            Remove-Item -LiteralPath $entry.Backup
        }
        if ($helperWritten -and (Test-Path -LiteralPath $helperPath)) { Remove-Item -LiteralPath $helperPath }
        throw
    }
    Write-Output 'ESP32-audioI2S 4.0.0 reserve patch: patched; restart Arduino IDE and rebuild'
} catch {
    [Console]::Error.WriteLine(('ESP32-audioI2S reserve patch: ' + $_.Exception.Message))
    exit 1
}
