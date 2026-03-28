[CmdletBinding()]
param(
    [string]$ALDER_REPO = "nruimveld7/ALDER",
    [string]$ALDER_VERSION = "latest",
    [string]$ALDER_ARCH = "X64"
)

$ErrorActionPreference = "Stop"

function Fail {
    param([string]$Message)
    Write-Error $Message
    exit 1
}

function Get-GitHubHeaders {
    $headers = @{
        "User-Agent" = "WILLOW-ALDER-Fetcher"
    }

    if ($env:GITHUB_TOKEN) {
        $headers["Authorization"] = "Bearer $($env:GITHUB_TOKEN)"
    }

    return $headers
}

function Invoke-WebRequestStrict {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Uri,
        [string]$OutFile
    )

    $params = @{
        Uri = $Uri
        Headers = (Get-GitHubHeaders)
        UseBasicParsing = $true
        ErrorAction = "Stop"
    }

    if ($OutFile) {
        $params["OutFile"] = $OutFile
    }

    try {
        return Invoke-WebRequest @params
    }
    catch {
        $statusCode = $null
        try {
            $statusCode = [int]$_.Exception.Response.StatusCode
        }
        catch {
            $statusCode = $null
        }

        if ($statusCode) {
            Fail "HTTP $statusCode while requesting '$Uri'. If this release/repo is private, set GITHUB_TOKEN and retry."
        }

        Fail "Request failed for '$Uri': $($_.Exception.Message)"
    }
}

function Get-AssetExtension {
    param([object]$Asset)

    $ext = $null
    if ($Asset.PSObject.Properties.Name -contains "extension") {
        $ext = [string]$Asset.extension
    }

    if (-not $ext -and ($Asset.PSObject.Properties.Name -contains "name")) {
        $ext = [System.IO.Path]::GetExtension([string]$Asset.name)
    }

    if (-not $ext -and ($Asset.PSObject.Properties.Name -contains "filename")) {
        $ext = [System.IO.Path]::GetExtension([string]$Asset.filename)
    }

    if (-not $ext -and ($Asset.PSObject.Properties.Name -contains "url")) {
        $urlPath = ([Uri][string]$Asset.url).AbsolutePath
        $ext = [System.IO.Path]::GetExtension($urlPath)
    }

    if (-not $ext) {
        $ext = ""
    }

    return $ext.ToLowerInvariant()
}

function Resolve-AssetUrl {
    param(
        [object]$Asset,
        [string]$Repo,
        [string]$Version
    )

    foreach ($prop in @("download_url", "browser_download_url", "url")) {
        if ($Asset.PSObject.Properties.Name -contains $prop) {
            $value = [string]$Asset.$prop
            if ($value) {
                return $value
            }
        }
    }

    $name = $null
    foreach ($nameProp in @("name", "filename")) {
        if ($Asset.PSObject.Properties.Name -contains $nameProp) {
            $candidate = [string]$Asset.$nameProp
            if ($candidate) {
                $name = $candidate
                break
            }
        }
    }

    if (-not $name) {
        return $null
    }

    return "https://github.com/$Repo/releases/download/$Version/$name"
}

function Resolve-AssetName {
    param([object]$Asset)

    foreach ($nameProp in @("name", "filename")) {
        if ($Asset.PSObject.Properties.Name -contains $nameProp) {
            $candidate = [string]$Asset.$nameProp
            if ($candidate) {
                return $candidate
            }
        }
    }

    return $null
}

function Resolve-ReleaseVersion {
    param(
        [string]$Repo,
        [string]$RequestedVersion
    )

    if ($RequestedVersion -and $RequestedVersion.ToLowerInvariant() -ne "latest") {
        return $RequestedVersion
    }

    $latestReleaseUri = "https://api.github.com/repos/$Repo/releases/latest"
    Write-Host "Resolving latest release from: $latestReleaseUri"
    $latestReleaseResponse = Invoke-WebRequestStrict -Uri $latestReleaseUri
    if (-not $latestReleaseResponse.Content) {
        Fail "Latest release request succeeded but content was empty."
    }

    $latestRelease = $latestReleaseResponse.Content | ConvertFrom-Json
    if (-not $latestRelease -or -not $latestRelease.tag_name) {
        Fail "Failed to resolve the latest release tag for '$Repo'."
    }

    return [string]$latestRelease.tag_name
}

function Normalize-ExpectedSha256 {
    param([string]$Value)

    if (-not $Value) {
        return $null
    }

    $trimmed = $Value.Trim()
    if ($trimmed -match "^sha256[:=](.+)$") {
        $trimmed = $Matches[1]
    }

    $trimmed = $trimmed.Trim().ToLowerInvariant()

    if ($trimmed -notmatch "^[0-9a-f]{64}$") {
        return $null
    }

    return $trimmed
}

try {
    if (-not $ALDER_REPO -or -not $ALDER_ARCH) {
        Fail "ALDER_REPO and ALDER_ARCH must be non-empty."
    }

    $resolvedVersion = Resolve-ReleaseVersion -Repo $ALDER_REPO -RequestedVersion $ALDER_VERSION

    $manifestUrl = "https://github.com/$ALDER_REPO/releases/download/$resolvedVersion/release-manifest.json"
    Write-Host "Downloading manifest: $manifestUrl"

    $manifestRaw = Invoke-WebRequestStrict -Uri $manifestUrl
    if (-not $manifestRaw.Content) {
        Fail "Manifest download succeeded but content was empty."
    }

    $manifestText = $null
    if ($manifestRaw.Content -is [byte[]]) {
        $manifestText = [System.Text.Encoding]::UTF8.GetString($manifestRaw.Content)
    } else {
        $manifestText = [string]$manifestRaw.Content
    }

    $manifest = $manifestText | ConvertFrom-Json
    if (-not $manifest) {
        Fail "Failed to parse release-manifest.json as JSON."
    }

    $assets = $null
    if ($manifest -is [System.Array]) {
        $assets = $manifest
    } elseif ($manifest.PSObject.Properties.Name -contains "assets") {
        $assets = $manifest.assets
    }

    if (-not $assets) {
        Fail "Manifest does not contain an assets list."
    }

    $selected = $assets | Where-Object {
        ($_.PSObject.Properties.Name -contains "os") -and
        ([string]$_.os -ieq "Windows") -and
        ($_.PSObject.Properties.Name -contains "arch") -and
        ([string]$_.arch -ieq $ALDER_ARCH) -and
        ((Get-AssetExtension -Asset $_) -in @(".exe", ".msi"))
    } | Select-Object -First 1

    if (-not $selected) {
        Fail "No Windows asset found for arch '$ALDER_ARCH' with extension .exe or .msi in release '$resolvedVersion'."
    }

    $assetName = Resolve-AssetName -Asset $selected
    if (-not $assetName) {
        $assetName = "alder-installer$(Get-AssetExtension -Asset $selected)"
    }

    $assetUrl = Resolve-AssetUrl -Asset $selected -Repo $ALDER_REPO -Version $resolvedVersion
    if (-not $assetUrl) {
        Fail "Could not resolve a download URL for the selected asset."
    }

    $expectedSha = $null
    foreach ($shaProp in @("sha256", "sha256sum", "sha256_hash", "checksum")) {
        if ($selected.PSObject.Properties.Name -contains $shaProp) {
            $candidate = Normalize-ExpectedSha256 -Value ([string]$selected.$shaProp)
            if ($candidate) {
                $expectedSha = $candidate
                break
            }
        }
    }

    if (-not $expectedSha -and ($manifest.PSObject.Properties.Name -contains "checksums")) {
        $checksums = $manifest.checksums
        foreach ($key in @($assetName, $assetUrl)) {
            if ($checksums.PSObject.Properties.Name -contains $key) {
                $candidate = Normalize-ExpectedSha256 -Value ([string]$checksums.$key)
                if ($candidate) {
                    $expectedSha = $candidate
                    break
                }
            }
        }
    }

    if (-not $expectedSha) {
        Fail "No valid SHA256 was found in the manifest for asset '$assetName'."
    }

    $downloadDir = Join-Path $env:TEMP "alder\$resolvedVersion\$ALDER_ARCH"
    New-Item -ItemType Directory -Path $downloadDir -Force | Out-Null
    $assetPath = Join-Path $downloadDir $assetName

    Write-Host "Downloading asset: $assetUrl"
    Invoke-WebRequestStrict -Uri $assetUrl -OutFile $assetPath | Out-Null

    if (-not (Test-Path -LiteralPath $assetPath)) {
        Fail "Asset download failed: file was not created at '$assetPath'."
    }

    $actualSha = (Get-FileHash -Path $assetPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualSha -ne $expectedSha) {
        Fail "SHA256 mismatch for '$assetName'. Expected '$expectedSha' but got '$actualSha'."
    }

    $extension = [System.IO.Path]::GetExtension($assetPath).ToLowerInvariant()
    Write-Host "SHA256 verified. Launching $assetName..."

    if ($extension -eq ".exe") {
        $process = Start-Process -FilePath $assetPath -Wait -PassThru
        if ($process.ExitCode -ne 0) {
            Fail "Executable failed with exit code $($process.ExitCode)."
        }
    } elseif ($extension -eq ".msi") {
        $msiProcess = Start-Process -FilePath "msiexec.exe" -ArgumentList @("/i", $assetPath, "/qn") -Wait -PassThru
        if ($msiProcess.ExitCode -ne 0) {
            Fail "MSI installation failed with exit code $($msiProcess.ExitCode)."
        }
    } else {
        Fail "Unsupported file type '$extension'. Expected .exe or .msi."
    }

    Write-Host "ALDER setup completed successfully."
    exit 0
}
catch {
    Fail $_.Exception.Message
}
