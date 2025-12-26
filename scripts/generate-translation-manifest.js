/**
 * Translation Manifest Generator
 *
 * This script scans the English docs directory and Urdu i18n directory
 * to generate a JSON manifest mapping each document to its translation availability.
 *
 * Output: frontend/src/translation-manifest.json
 *
 * Usage: node scripts/generate-translation-manifest.js
 */

const fs = require('fs');
const path = require('path');

const docsDir = path.resolve(__dirname, '../frontend/docs');
const urduDir = path.resolve(__dirname, '../frontend/i18n/ur/docusaurus-plugin-content-docs/current');
const outputPath = path.resolve(__dirname, '../frontend/src/translation-manifest.json');

const manifest = {};

/**
 * Recursively scan docs directory and check for Urdu translations
 * @param {string} dir - Directory to scan
 * @param {string} basePath - Relative path from docs root
 */
function scanDocs(dir, basePath = '') {
  if (!fs.existsSync(dir)) {
    console.warn(`⚠️  Warning: Directory not found: ${dir}`);
    return;
  }

  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      // Recursively scan subdirectories
      scanDocs(filePath, path.join(basePath, file));
    } else if (file.match(/\.mdx?$/)) {
      // Check if markdown file (.md or .mdx)
      const relativePath = path.join(basePath, file).replace(/\\/g, '/');
      const urduPath = path.join(urduDir, relativePath);

      // Check if Urdu translation exists
      manifest[relativePath] = fs.existsSync(urduPath);
    }
  });
}

// Main execution
console.log('🔍 Scanning docs for translation availability...\n');
console.log(`📂 English docs: ${docsDir}`);
console.log(`📂 Urdu i18n: ${urduDir}\n`);

scanDocs(docsDir);

// Calculate statistics
const totalFiles = Object.keys(manifest).length;
const translatedFiles = Object.values(manifest).filter(Boolean).length;
const untranslatedFiles = totalFiles - translatedFiles;

// Write manifest to JSON file
fs.writeFileSync(outputPath, JSON.stringify(manifest, null, 2));

// Print results
console.log('✅ Translation manifest generated successfully!\n');
console.log(`📊 Statistics:`);
console.log(`   Total files: ${totalFiles}`);
console.log(`   Translated to Urdu: ${translatedFiles}`);
console.log(`   Not yet translated: ${untranslatedFiles}`);
console.log(`   Translation coverage: ${totalFiles > 0 ? Math.round((translatedFiles / totalFiles) * 100) : 0}%\n`);
console.log(`📄 Output: ${outputPath}`);
