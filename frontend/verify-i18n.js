/**
 * Docusaurus i18n Verification Script
 *
 * This script verifies that:
 * 1. Both locale builds exist (en and ur)
 * 2. Translation manifest is valid
 * 3. Urdu translation files exist for marked chapters
 * 4. Build artifacts are correctly structured
 *
 * Usage: node verify-i18n.js
 */

const fs = require('fs');
const path = require('path');

const colors = {
  reset: '\x1b[0m',
  green: '\x1b[32m',
  red: '\x1b[31m',
  yellow: '\x1b[33m',
  blue: '\x1b[36m',
};

const log = {
  success: (msg) => console.log(`${colors.green}✓${colors.reset} ${msg}`),
  error: (msg) => console.log(`${colors.red}✗${colors.reset} ${msg}`),
  warning: (msg) => console.log(`${colors.yellow}⚠${colors.reset} ${msg}`),
  info: (msg) => console.log(`${colors.blue}ℹ${colors.reset} ${msg}`),
};

let checks = 0;
let passed = 0;
let failed = 0;

function check(condition, successMsg, errorMsg) {
  checks++;
  if (condition) {
    log.success(successMsg);
    passed++;
    return true;
  } else {
    log.error(errorMsg);
    failed++;
    return false;
  }
}

console.log('\n=== Docusaurus i18n Verification ===\n');

// Check 1: Build directory exists
const buildDir = path.join(__dirname, 'build');
check(
  fs.existsSync(buildDir),
  'Build directory exists',
  'Build directory missing - run "npm run build" first'
);

// Check 2: Urdu build directory exists
const urBuildDir = path.join(__dirname, 'build', 'ur');
check(
  fs.existsSync(urBuildDir),
  'Urdu build directory exists',
  'Urdu build directory missing - check docusaurus.config.ts i18n config'
);

// Check 3: Translation manifest exists
const manifestPath = path.join(__dirname, 'src', 'translation-manifest.json');
check(
  fs.existsSync(manifestPath),
  'Translation manifest exists',
  'Translation manifest missing - run "node scripts/generate-translation-manifest.js"'
);

// Check 4: Validate manifest structure
if (fs.existsSync(manifestPath)) {
  try {
    const manifest = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
    const entries = Object.keys(manifest);
    check(
      entries.length > 0,
      `Translation manifest has ${entries.length} entries`,
      'Translation manifest is empty'
    );

    // Check 5: Validate manifest entries
    const translatedCount = entries.filter(key => manifest[key] === true).length;
    log.info(`${translatedCount} of ${entries.length} chapters marked as translated`);

    // Check 6: Verify translated files exist
    entries.forEach((docPath) => {
      if (manifest[docPath] === true) {
        const urFilePath = path.join(
          __dirname,
          'i18n',
          'ur',
          'docusaurus-plugin-content-docs',
          'current',
          docPath
        );
        check(
          fs.existsSync(urFilePath),
          `Urdu translation exists: ${docPath}`,
          `Urdu translation missing: ${docPath} (marked as true in manifest)`
        );

        // Check 7: Verify build output exists
        const buildPath = path
          .join(__dirname, 'build', 'ur', 'docs', docPath)
          .replace('.mdx', '')
          .replace('.md', '') + '/index.html';
        check(
          fs.existsSync(buildPath),
          `Build output exists: ${docPath}`,
          `Build output missing: ${buildPath}`
        );
      }
    });
  } catch (error) {
    log.error(`Failed to parse translation manifest: ${error.message}`);
    failed++;
  }
}

// Check 8: Verify config file
const configPath = path.join(__dirname, 'docusaurus.config.ts');
if (fs.existsSync(configPath)) {
  const config = fs.readFileSync(configPath, 'utf8');
  check(
    config.includes('locales: ["en", "ur"]'),
    'Docusaurus config has correct locales',
    'Docusaurus config missing or incorrect locale configuration'
  );
  check(
    config.includes('direction: "rtl"'),
    'Urdu locale configured as RTL',
    'Urdu RTL direction not configured'
  );
}

// Check 9: Verify hook implementation
const hookPath = path.join(__dirname, 'src', 'hooks', 'useLanguagePreference.ts');
if (fs.existsSync(hookPath)) {
  const hookContent = fs.readFileSync(hookPath, 'utf8');
  check(
    hookContent.includes('useDocusaurusContext'),
    'Hook uses Docusaurus context (FIXED)',
    'Hook not using Docusaurus context - language detection may be wrong'
  );
  check(
    hookContent.includes('i18n.currentLocale'),
    'Hook reads from i18n.currentLocale (CORRECT)',
    'Hook not reading from i18n.currentLocale'
  );
  check(
    !hookContent.includes('localStorage.getItem'),
    'Hook no longer uses localStorage (CORRECT)',
    'Hook still using localStorage - may cause wrong button text'
  );
}

// Summary
console.log('\n=== Verification Summary ===\n');
log.info(`Total checks: ${checks}`);
log.success(`Passed: ${passed}`);
if (failed > 0) {
  log.error(`Failed: ${failed}`);
}

if (failed === 0) {
  console.log(`\n${colors.green}All checks passed! Your i18n implementation is correct.${colors.reset}`);
  console.log('\nNext steps:');
  console.log('1. Clear browser localStorage: localStorage.clear() in console');
  console.log('2. Test English: npm run start');
  console.log('3. Test Urdu: npm run start -- --locale ur');
  console.log('4. Deploy to production: vercel --prod');
  process.exit(0);
} else {
  console.log(`\n${colors.red}Some checks failed. Review errors above.${colors.reset}`);
  console.log('\nTroubleshooting:');
  console.log('1. Rebuild: npm run build');
  console.log('2. Regenerate manifest: node scripts/generate-translation-manifest.js');
  console.log('3. Check FIXES-APPLIED.md for details');
  process.exit(1);
}
