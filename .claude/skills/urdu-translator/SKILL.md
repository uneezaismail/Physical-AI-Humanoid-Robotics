# Skill: urdu-translator

## Purpose
The urdu-translator skill automates the translation of Docusaurus content from English to Urdu, enabling multilingual documentation for the Physical AI & Humanoid Robotics textbook. This skill ensures proper RTL (Right-to-Left) language support, preserves markdown formatting, and maintains consistency across language versions.

## Capabilities
This skill can:
- Configure Docusaurus i18n for Urdu (ur) locale with RTL support
- Generate and translate Docusaurus theme/plugin UI strings (navbar, footer, etc.)
- Translate markdown documentation files while preserving structure and frontmatter
- Set up proper RTL layout configuration for Urdu language
- Validate translation file structure and completeness
- Provide testing and deployment guidance

## Prerequisites & Initial Setup

### 1. Docusaurus Configuration (docusaurus.config.ts/js)
The agent MUST verify/configure the i18n section in `docs/docusaurus.config.ts`:

```typescript
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    path: 'i18n',
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو', // Urdu in Urdu script
        direction: 'rtl', // CRITICAL: RTL for Urdu
        htmlLang: 'ur-PK',
      },
    },
  },
  // ... rest of config
};
```

**Key RTL Configuration Points:**
- `direction: 'rtl'` is MANDATORY for Urdu - this automatically flips the theme (sidebar to right, TOC to left)
- `htmlLang: 'ur-PK'` sets proper HTML lang attribute
- `label: 'اردو'` displays native script in language switcher

### 2. Generate Initial Translation Files
Before translating, run this command to generate JSON files for UI elements:

```bash
cd docs
npm run write-translations -- --locale ur
```

**Expected Output:**
```
1 translations written at i18n/ur/code.json
11 translations written at i18n/ur/docusaurus-theme-classic/footer.json
4 translations written at i18n/ur/docusaurus-theme-classic/navbar.json
3 translations written at i18n/ur/docusaurus-plugin-content-docs/current.json
```

## File Structure Overview

The agent should work with this i18n structure:

```
docs/
├── docs/                          # Source English content
│   ├── overview.md
│   ├── chapter-01-foundations/
│   │   ├── lesson-01-intro.md
│   │   └── ...
│   └── ...
├── i18n/
│   └── ur/                        # Urdu translations root
│       ├── code.json              # General UI strings
│       ├── docusaurus-theme-classic/
│       │   ├── navbar.json        # Navbar translations
│       │   └── footer.json        # Footer translations
│       └── docusaurus-plugin-content-docs/
│           └── current/           # Current version docs
│               ├── overview.md    # Translated markdown
│               ├── chapter-01-foundations/
│               │   ├── lesson-01-intro.md
│               │   └── ...
│               └── current.json   # Plugin metadata
└── docusaurus.config.ts
```

## Translation Workflow

### Phase 1: JSON UI String Translation

**Target Files:**
- `i18n/ur/code.json`
- `i18n/ur/docusaurus-theme-classic/navbar.json`
- `i18n/ur/docusaurus-theme-classic/footer.json`
- `i18n/ur/docusaurus-plugin-content-docs/current.json`

**Process:**
1. Read each JSON file
2. For each key-value pair, translate the English `message` to Urdu
3. Preserve JSON structure exactly
4. Keep `description` field in English (it's for translators, not displayed)

**Example Translation:**

Before:
```json
{
  "theme.docs.sidebar.navAriaLabel": {
    "message": "Docs sidebar",
    "description": "The ARIA label for the sidebar navigation"
  }
}
```

After:
```json
{
  "theme.docs.sidebar.navAriaLabel": {
    "message": "دستاویزات سائیڈبار",
    "description": "The ARIA label for the sidebar navigation"
  }
}
```

### Phase 2: Markdown Content Translation

**Target Directory:** `i18n/ur/docusaurus-plugin-content-docs/current/`

**Critical Rules:**
1. **Mirror Directory Structure:** Create exact same folder hierarchy as `docs/docs/`
2. **Preserve Frontmatter Keys:** Only translate VALUES, never KEYS
3. **Preserve Markdown Syntax:** Keep all formatting (headings, lists, code blocks, links, etc.)
4. **Do NOT Translate:**
   - Code blocks content
   - File paths, URLs
   - Import/export statements
   - Component names (e.g., `<Tabs>`, `<CodeBlock>`)
   - Frontmatter keys (e.g., `title:`, `sidebar_label:`)
   - Image paths
   - Anchor IDs

**Translation Process for Each File:**

1. **Create Urdu File:**
   ```bash
   # Example: translating docs/docs/overview.md
   # Create: i18n/ur/docusaurus-plugin-content-docs/current/overview.md
   ```

2. **Copy Full Content:**
   - Copy entire file from source to i18n location

3. **Translate Content Systematically:**

   **Frontmatter (YAML):**
   ```yaml
   ---
   id: overview              # DO NOT translate
   title: Overview           # TRANSLATE value only
   sidebar_label: Overview   # TRANSLATE value only
   sidebar_position: 1       # DO NOT translate
   ---
   ```

   Becomes:
   ```yaml
   ---
   id: overview
   title: جائزہ
   sidebar_label: جائزہ
   sidebar_position: 1
   ---
   ```

   **Headings:**
   ```markdown
   ## What is Physical AI?
   ```

   Becomes:
   ```markdown
   ## فزیکل AI کیا ہے؟
   ```

   **Paragraphs:**
   - Translate all text content
   - Preserve line breaks and paragraph structure

   **Lists:**
   ```markdown
   - First item
   - Second item
   ```

   Becomes:
   ```markdown
   - پہلی چیز
   - دوسری چیز
   ```

   **Code Blocks (DO NOT TRANSLATE CODE):**
   ```markdown
   ```python
   import rclpy
   from rclpy.node import Node

   class MyNode(Node):
       pass
   ```
   ```

   **Keep code exactly as is!** Only translate surrounding text.

   **Links:**
   ```markdown
   [Learn more](./introduction.md)
   ```

   Becomes:
   ```markdown
   [مزید جانیں](./introduction.md)  # Text translated, path preserved
   ```

   **Admonitions/Callouts:**
   ```markdown
   :::tip Pro Tip
   This is helpful information.
   :::
   ```

   Becomes:
   ```markdown
   :::tip پرو ٹپ
   یہ مددگار معلومات ہے۔
   :::
   ```

4. **Quality Checks for Each File:**
   - [ ] Frontmatter YAML is valid (no syntax errors)
   - [ ] All markdown syntax preserved (headings use #, lists use -, etc.)
   - [ ] Code blocks untouched
   - [ ] Links functional (paths unchanged)
   - [ ] No English text in translated sections
   - [ ] No broken markdown rendering

## Testing & Validation

### Local Testing
After translation, test the Urdu locale:

```bash
cd docs
npm run start -- --locale ur
```

**Access:** http://localhost:3000/ur/

### Validation Checklist
The agent should verify:

**Visual/RTL Validation:**
- [ ] Site displays in RTL (sidebar on right, TOC on left)
- [ ] Text flows right-to-left
- [ ] Language switcher shows "اردو" label
- [ ] No layout breaks or overlapping elements

**Content Validation:**
- [ ] All pages accessible at `/ur/` prefix
- [ ] Markdown renders correctly (headings, lists, code blocks)
- [ ] Frontmatter values translated (page titles, sidebar labels)
- [ ] Code blocks display properly and are NOT translated
- [ ] Links work (internal navigation functional)
- [ ] Images display correctly

**UI Validation:**
- [ ] Navbar translated
- [ ] Footer translated
- [ ] Sidebar navigation translated
- [ ] Search functionality works (if enabled)
- [ ] Theme labels translated (Next, Previous, etc.)

### Build Testing
Test production build with all locales:

```bash
cd docs
npm run build
```

This builds both `en` and `ur` locales. Verify no build errors.

## Input (to the skill)

The agent accepts:
- **Target files:** Specific markdown files to translate (or "all")
- **Translation mode:**
  - `json-only`: Only translate UI strings
  - `docs-only`: Only translate markdown docs
  - `full`: Complete translation (JSON + markdown)
- **Incremental mode:** Skip already-translated files (default: false)

**Example Invocations:**
```
"Translate all content to Urdu"
"Translate chapter 1 files to Urdu"
"Update Urdu UI strings only"
"Translate docs/overview.md to Urdu"
```

## Output (from the skill)

The agent provides:

1. **Translation Summary Report:**
   ```
   ✅ Urdu Translation Complete

   📄 JSON Files Translated: 4
   - i18n/ur/code.json (15 strings)
   - i18n/ur/docusaurus-theme-classic/navbar.json (8 strings)
   - i18n/ur/docusaurus-theme-classic/footer.json (12 strings)
   - i18n/ur/docusaurus-plugin-content-docs/current.json (5 strings)

   📝 Markdown Files Translated: 23
   - i18n/ur/docusaurus-plugin-content-docs/current/overview.md
   - i18n/ur/docusaurus-plugin-content-docs/current/chapter-01-foundations/...
   [... full list ...]

   ⚠️  Issues Encountered: 0
   ```

2. **Testing Instructions:**
   ```
   🧪 Test Locally:
   cd docs && npm run start -- --locale ur
   Visit: http://localhost:3000/ur/

   🏗️  Build Production:
   cd docs && npm run build

   📋 Validation Checklist:
   - Verify RTL layout (sidebar right, content flows RTL)
   - Check language switcher displays "اردو"
   - Confirm all pages load without errors
   - Validate code blocks render correctly
   - Test internal links navigate properly
   ```

3. **Deployment Readiness:**
   - Confirmation that all files created/updated
   - Build success status
   - Any warnings or manual review needed

## Advanced Features

### Incremental Translation
When adding new content:
1. Add/update English markdown in `docs/docs/`
2. Run `npm run write-translations -- --locale ur` to update JSON
3. Copy new/modified markdown to `i18n/ur/docusaurus-plugin-content-docs/current/`
4. Translate only new content

### Translation Override
To regenerate JSON files from scratch:
```bash
npm run write-translations -- --locale ur --override
```

### Multi-Domain Deployment (Advanced)
For separate Urdu domain, configure in `localeConfigs`:
```typescript
ur: {
  label: 'اردو',
  direction: 'rtl',
  htmlLang: 'ur-PK',
  url: 'https://ur.example.com',  // Separate domain
  baseUrl: '/',
}
```

## Translation Quality Guidelines

### Urdu Translation Best Practices
1. **Terminology Consistency:**
   - Maintain consistent technical term translations
   - Create a glossary for recurring terms (e.g., "Physical AI" → "فزیکل AI")

2. **Urdu Script Standards:**
   - Use proper Urdu Unicode characters
   - Apply correct diacritical marks where necessary
   - Follow Urdu grammar and sentence structure

3. **Technical Content:**
   - Keep English technical terms where commonly used (e.g., "ROS 2", "API", "GPU")
   - Translate explanatory text fully
   - Balance between localization and technical accuracy

4. **Cultural Adaptation:**
   - Use Pakistani Urdu conventions
   - Adapt examples if culturally relevant
   - Maintain professional educational tone

## Troubleshooting

### Common Issues

**Issue:** RTL not working
- **Fix:** Verify `direction: 'rtl'` in `docusaurus.config.ts` → `i18n.localeConfigs.ur`

**Issue:** Translation files not found
- **Fix:** Run `npm run write-translations -- --locale ur` first

**Issue:** Broken frontmatter after translation
- **Fix:** Check YAML syntax - keys should NOT be translated, only values

**Issue:** Code blocks displaying incorrectly
- **Fix:** Ensure code block content is NOT translated - preserve exactly as source

**Issue:** Build fails with locale errors
- **Fix:** Verify `ur` is in `i18n.locales` array in config

## Usage Notes

- **Prerequisites:** Docusaurus 3.x with i18n configured
- **Translation Model:** Uses Claude's multilingual capabilities for English→Urdu
- **Manual Review:** Recommended for critical educational content and technical accuracy
- **Image Localization:** Not handled by this skill - requires manual asset management
- **Search Localization:** If using Algolia/DocSearch, configure separately for Urdu
- **Performance:** Translations are static - no runtime overhead

## References

- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [RTL Language Support](https://docusaurus.io/docs/i18n/introduction)
- Urdu Locale Code: `ur` (ISO 639-1)
- HTML Lang Tag: `ur-PK` (Urdu - Pakistan)