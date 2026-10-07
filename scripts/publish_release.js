#!/usr/bin/env node
/**
 * Automated GitHub Release Publisher for Smart AC Controller Firmware.
 * 
 * Usage:
 *   node scripts/publish_release.js [version]
 *   Example: node scripts/publish_release.js v1.3.36
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

function getGitToken() {
  if (process.env.GITHUB_TOKEN) return process.env.GITHUB_TOKEN.trim();
  try {
    const creds = execSync('printf "protocol=https\\nhost=github.com\\n\\n" | git credential fill 2>/dev/null', { encoding: 'utf8' });
    const match = creds.match(/^password=(.+)$/m);
    if (match) return match[1].trim();
  } catch (_) {}
  return null;
}

function getRepoName() {
  try {
    const remote = execSync('git remote get-url origin', { encoding: 'utf8' }).trim();
    const match = remote.match(/github\.com[:/]([^/]+)\/([^/.]+)/);
    if (match) return `${match[1]}/${match[2]}`;
  } catch (_) {}
  return 'vinideep/smart-ac-controller-firmware';
}

function getNextVersion() {
  try {
    const count = execSync('git rev-list --count HEAD', { encoding: 'utf8' }).trim();
    return `v1.3.${count}`;
  } catch (_) {
    return 'v1.3.36';
  }
}

function findFirmwareBinary() {
  const candidates = [
    path.resolve(__dirname, '../.pio/build/esp32dev/firmware.bin'),
    path.resolve(__dirname, '../../data/firmware.bin'),
    path.resolve(process.cwd(), 'firmware/.pio/build/esp32dev/firmware.bin'),
    path.resolve(process.cwd(), 'data/firmware.bin'),
  ];
  for (const c of candidates) {
    if (fs.existsSync(c)) {
      const stat = fs.statSync(c);
      if (stat.size >= 50000) {
        const buf = fs.readFileSync(c);
        if (buf[0] === 0xE9) {
          return { path: c, buffer: buf };
        }
      }
    }
  }
  return null;
}

async function main() {
  const token = getGitToken();
  if (!token) {
    console.error('Error: No GitHub token found in GITHUB_TOKEN env or git credential store.');
    process.exit(1);
  }

  const repo = getRepoName();
  const version = process.argv[2] || getNextVersion();
  console.log(`Publishing release for repository: ${repo}`);
  console.log(`Target version: ${version}`);

  const fw = findFirmwareBinary();
  if (!fw) {
    console.error('Error: No valid ESP32 firmware binary found (must be >= 50KB with magic byte 0xE9).');
    process.exit(1);
  }
  console.log(`Found valid ESP32 binary at: ${fw.path} (${fw.buffer.length} bytes)`);

  // 1. Create Release on GitHub
  console.log(`Creating GitHub release ${version}...`);
  const relRes = await fetch(`https://api.github.com/repos/${repo}/releases`, {
    method: 'POST',
    headers: {
      'Authorization': `token ${token}`,
      'Accept': 'application/vnd.github.v3+json',
      'Content-Type': 'application/json',
      'User-Agent': 'SmartAC-Firmware-Publisher'
    },
    body: JSON.stringify({
      tag_name: version,
      target_commitish: 'main',
      name: `${version} Wireless Cloud Release`,
      body: `Automated OTA release for AzureEssence Smart AC Controller.\n- Dual-bank OTA partition support\n- Psychrometric climate engine\n- Circadian sleep automation\n- ESP32 bootloader magic byte verified (0xE9)`,
      draft: false,
      prerelease: false
    })
  });

  const relData = await relRes.json();
  if (!relRes.ok) {
    console.error(`Failed to create release (HTTP ${relRes.status}):`, relData.message || relData);
    process.exit(1);
  }

  console.log(`Release created: ${relData.html_url}`);
  const uploadUrl = relData.upload_url.replace(/\{(\?.*)?\}/, '');

  // 2. Upload firmware.bin asset
  console.log('Uploading firmware.bin asset...');
  const uploadRes = await fetch(`${uploadUrl}?name=firmware.bin`, {
    method: 'POST',
    headers: {
      'Authorization': `token ${token}`,
      'Content-Type': 'application/octet-stream',
      'Content-Length': fw.buffer.length,
      'User-Agent': 'SmartAC-Firmware-Publisher'
    },
    body: fw.buffer
  });

  const uploadData = await uploadRes.json();
  if (!uploadRes.ok) {
    console.error(`Failed to upload asset (HTTP ${uploadRes.status}):`, uploadData.message || uploadData);
    process.exit(1);
  }

  console.log(`SUCCESS! Firmware uploaded.`);
  console.log(`Download URL: ${uploadData.browser_download_url}`);
}

main().catch(err => {
  console.error('Fatal error:', err);
  process.exit(1);
});
