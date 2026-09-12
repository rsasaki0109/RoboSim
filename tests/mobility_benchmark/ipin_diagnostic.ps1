# Fixed, read-only source diagnostic. Data/output must be explicitly located on external storage.
param(
    [Parameter(Mandatory=$true)][string]$Archive,
    [Parameter(Mandatory=$true)][string]$Executable,
    [Parameter(Mandatory=$true)][string]$OutputPath,
    [ValidateSet('FullInterior','TimeSegments')][string]$Protocol = 'FullInterior'
)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $OutputPath) { throw 'Refusing to overwrite evidence' }
$expected = @{
    'acceleration_Allan.csv' = '1357f596e2089f0ad3be543c337a00624802af248fbb2bd7c12b709773cd53cc'
    'rotation_Allan.csv' = 'abc06f17b266bf441f4b516ad3790c49b2aebbfe0fe506a39654c27e785d5512'
}
$reports = [System.Collections.Generic.List[object]]::new()
$firstOutputs = @{}
$firstEndpoints = if ($Protocol -eq 'TimeSegments') {
    @('259556000000', '280356000000', '301156000000')
} else { @('259556000000') }
$endpoints = if ($Protocol -eq 'TimeSegments') { 10800 } else { 52963 }
$sourceHashes = @{}
foreach ($source in @('crates/rne_sensor/src/allan/timed.rs',
    'tests/mobility_benchmark/src/recorded_ipin.rs',
    'tests/mobility_benchmark/examples/ipin_time_deviation.rs',
    'tests/mobility_benchmark/ipin_diagnostic.ps1')) {
    $sourceHashes[$source] = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash.ToLowerInvariant()
}
$exeHash = (Get-FileHash -LiteralPath $Executable -Algorithm SHA256).Hash.ToLowerInvariant()
$zip = [System.IO.Compression.ZipFile]::OpenRead($Archive)
try {
    for ($repeat=0; $repeat -lt 2; $repeat++) {
        foreach ($name in @('acceleration_Allan.csv','rotation_Allan.csv')) {
          foreach ($firstEndpoint in $firstEndpoints) {
            for ($axis=0; $axis -lt 3; $axis++) {
                $info = [System.Diagnostics.ProcessStartInfo]::new($Executable)
                $info.UseShellExecute=$false
                $info.CreateNoWindow=$true
                $info.RedirectStandardInput=$true
                $info.RedirectStandardOutput=$true
                $info.RedirectStandardError=$true
                foreach ($argument in @('-', "$axis", '1000000', $firstEndpoint, '1000000', "$endpoints")) {
                    $info.ArgumentList.Add($argument)
                }
                $process = [System.Diagnostics.Process]::Start($info)
                try {
                    $stdout = $process.StandardOutput.ReadToEndAsync()
                    $stderr = $process.StandardError.ReadToEndAsync()
                    $entry = $zip.GetEntry($name)
                    if ($null -eq $entry) { throw "Missing entry $name" }
                    $inputStream = $entry.Open()
                    try { $inputStream.CopyTo($process.StandardInput.BaseStream) }
                    finally { $inputStream.Dispose(); $process.StandardInput.Close() }
                    $process.WaitForExit()
                    $text = $stdout.GetAwaiter().GetResult()
                    $errorText = $stderr.GetAwaiter().GetResult()
                    if ($process.ExitCode -ne 0) { throw "Diagnostic failed: $errorText" }
                    $report = $text | ConvertFrom-Json
                    if ($report.source.source_sha256 -cne $expected[$name]) { throw 'Source hash mismatch' }
                    if ($report.axis -ne $axis -or $report.statistic.valid_pairs + $report.statistic.empty_pairs -ne $endpoints) {
                        throw 'Incomplete diagnostic result'
                    }
                    $key = "$name/$firstEndpoint/$axis"
                    if ($repeat -eq 0) {
                        $firstOutputs[$key] = $text
                        $reports.Add([pscustomobject]@{entry=$name; first_endpoint_us=$firstEndpoint; result=$report})
                    } elseif (-not [String]::Equals($firstOutputs[$key],$text,[StringComparison]::Ordinal)) {
                        throw "Non-reproducible output $key"
                    }
                    Write-Output "repeat=$repeat entry=$name start_us=$firstEndpoint axis=$axis variance=$($report.statistic.variance_source_units_squared)"
                } finally {
                    if (-not $process.HasExited) { $process.Kill(); $process.WaitForExit() }
                    $process.Dispose()
                }
            }
          }
        }
    }
} finally { $zip.Dispose() }
if ((Get-FileHash -LiteralPath $Executable -Algorithm SHA256).Hash.ToLowerInvariant() -cne $exeHash) {
    throw 'Executable changed during experiment'
}
foreach ($source in $sourceHashes.Keys) {
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash.ToLowerInvariant() -cne $sourceHashes[$source]) {
        throw 'Source changed during experiment'
    }
}
$result = [ordered]@{
    schema_version=1; protocol=$(if ($Protocol -eq 'TimeSegments') { 'ipin_equal_duration_time_segments_v1' } else { 'ipin_time_window_diagnostic_v1' }); repeats=2
    window_us=1000000; endpoint_period_us=1000000; endpoints=$endpoints; first_endpoints_us=$firstEndpoints
    byte_identical_repeats=$true; executable_sha256=$exeHash; source_code_sha256=$sourceHashes
    physical_calibration=$false; reports=$reports.ToArray()
}
$output = [System.IO.File]::Open($OutputPath,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write)
try {
    $bytes = [System.Text.UTF8Encoding]::new($false).GetBytes(($result | ConvertTo-Json -Depth 12))
    $output.Write($bytes,0,$bytes.Length)
    $output.Flush($true)
} finally { $output.Dispose() }
Get-FileHash -LiteralPath $OutputPath -Algorithm SHA256
