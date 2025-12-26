# Developer Quickstart: Interactive Urdu Translation Button

**Feature**: 002-urdu-translation
**Last Updated**: 2025-12-18

## Overview

This guide provides developers with everything needed to implement the Interactive Urdu Translation Button feature. Follow these steps sequentially for a smooth implementation experience.

---

## Prerequisites

- Node.js 18+ installed
- Familiarity with TypeScript, React, and Docusaurus
- Project cloned and dependencies installed (`npm install` in `frontend/`)
- Urdu translations pre-generated (via `urdu-translator` skill) in `frontend/i18n/ur/`

---

## Architecture at a Glance

```
┌─────────────────────────────────────────────────────────────┐
│                    User clicks button                        │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
          ┌────────────────────────┐
          │  LanguageSwitcher.tsx  │
          │  (React Component)     │
          └────────┬──────┬────────┘
                   │      │
         ┌─────────┘      └─────────┐
         ▼                          ▼
┌─────────────────┐        ┌─────────────────┐
│  localStorage   │        │  Docusaurus     │
│  (persist pref) │        │  Router         │
└─────────────────┘        │  (navigate)     │
                           └────────┬────────┘
                                    │
                                    ▼
                           ┌────────────────┐
                           │  /ur/docs/...  │
                           │  (Urdu locale) │
                           └────────────────┘
```

---

## Step-by-Step Implementation

### Step 1: Configure Docusaurus i18n (5 minutes)

**File**: `frontend/docusaurus.config.ts`

Add Urdu locale to i18n configuration:

```typescript
// frontend/docusaurus.config.ts
export default {
  // ... existing config
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],  // Add 'ur' here
    path: 'i18n',
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {  // NEW: Urdu configuration
        label: 'اردو',
        direction: 'rtl',  // RIGHT-TO-LEFT
        htmlLang: 'ur-PK',
      },
    },
  },
  // ... rest of config
};
```

**Verify**: Run `npm run start` and check browser console for no i18n errors.

---

### Step 2: Generate Translation Manifest (10 minutes)

**File**: `scripts/generate-translation-manifest.js` (create new file)

```javascript
// scripts/generate-translation-manifest.js
const fs = require('fs');
const path = require('path');

const docsDir = path.resolve(__dirname, '../frontend/docs');
const urduDir = path.resolve(__dirname, '../frontend/i18n/ur/docusaurus-plugin-content-docs/current');
const outputPath = path.resolve(__dirname, '../frontend/src/translation-manifest.json');

const manifest = {};

function scanDocs(dir, basePath = '') {
  if (!fs.existsSync(dir)) return;

  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      scanDocs(filePath, path.join(basePath, file));
    } else if (file.match(/\.mdx?$/)) {
      const relativePath = path.join(basePath, file).replace(/\\/g, '/');
      const urduPath = path.join(urduDir, relativePath);
      manifest[relativePath] = fs.existsSync(urduPath);
    }
  });
}

scanDocs(docsDir);

fs.writeFileSync(outputPath, JSON.stringify(manifest, null, 2));
console.log(`✅ Translation manifest generated: ${Object.keys(manifest).length} files mapped`);
```

**Run**:
```bash
node scripts/generate-translation-manifest.js
```

**Verify**: Check `frontend/src/translation-manifest.json` exists with chapter paths.

---

### Step 3: Create useLanguagePreference Hook (15 minutes)

**File**: `frontend/src/hooks/useLanguagePreference.ts` (create new)

```typescript
// frontend/src/hooks/useLanguagePreference.ts
import { useState, useEffect } from 'react';
import type { LanguageCode } from '../types/i18n';

const STORAGE_KEY = 'preferredLanguage';
const QUERY_PARAM = 'lang';

export function useLanguagePreference() {
  const [language, setLanguage] = useState<LanguageCode>('en');
  const [isLoading, setIsLoading] = useState(true);

  // Initialize from URL → localStorage → default
  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const urlLang = urlParams.get(QUERY_PARAM);

    if (urlLang === 'en' || urlLang === 'ur') {
      setLanguage(urlLang);
    } else {
      try {
        const stored = localStorage.getItem(STORAGE_KEY);
        if (stored === 'en' || stored === 'ur') {
          setLanguage(stored);
        }
      } catch (e) {
        console.warn('localStorage unavailable:', e);
      }
    }

    setIsLoading(false);
  }, []);

  const switchLanguage = (newLang: LanguageCode) => {
    setLanguage(newLang);

    // Persist to localStorage
    try {
      localStorage.setItem(STORAGE_KEY, newLang);
    } catch (e) {
      console.warn('Failed to save language preference:', e);
    }

    // Update URL parameter
    const url = new URL(window.location.href);
    url.searchParams.set(QUERY_PARAM, newLang);
    window.history.pushState({}, '', url.toString());
  };

  return { language, switchLanguage, isLoading };
}
```

---

### Step 4: Create LanguageSwitcher Component (20 minutes)

**File**: `frontend/src/components/LanguageSwitcher/LanguageSwitcher.tsx`

```typescript
// frontend/src/components/LanguageSwitcher/LanguageSwitcher.tsx
import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { useLanguagePreference } from '@/hooks/useLanguagePreference';
import translationManifest from '@/translation-manifest.json';
import styles from './LanguageSwitcher.module.css';

export function LanguageSwitcher() {
  const { language, switchLanguage } = useLanguagePreference();
  const history = useHistory();
  const location = useLocation();

  // Check if translation available for current page
  const currentDocPath = location.pathname.replace(/^\/?(ur\/)?docs\//, '');
  const translationAvailable = translationManifest[currentDocPath + '.md'] === true;

  const handleClick = () => {
    const newLang = language === 'en' ? 'ur' : 'en';
    switchLanguage(newLang);

    // Navigate to new locale
    const newPath = newLang === 'ur'
      ? `/ur${location.pathname}`
      : location.pathname.replace('/ur', '');

    history.push(newPath);
  };

  const isDisabled = !translationAvailable && language === 'en';
  const buttonText = language === 'en' ? 'Translate to Urdu' : 'Translate to English';
  const tooltip = isDisabled ? 'Urdu translation coming soon' : '';

  return (
    <button
      className={styles.languageSwitcher}
      onClick={handleClick}
      disabled={isDisabled}
      title={tooltip}
      aria-label={buttonText}
    >
      {buttonText}
    </button>
  );
}
```

**File**: `frontend/src/components/LanguageSwitcher/LanguageSwitcher.module.css`

```css
.languageSwitcher {
  padding: 8px 16px;
  font-size: 14px;
  font-weight: 500;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
  background-color: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  cursor: pointer;
  transition: all 0.2s ease;
  margin-bottom: 16px;
}

.languageSwitcher:hover:not(:disabled) {
  background-color: var(--ifm-color-primary);
  color: white;
  border-color: var(--ifm-color-primary);
}

.languageSwitcher:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

@media (max-width: 768px) {
  .languageSwitcher {
    font-size: 12px;
    padding: 6px 12px;
  }
}
```

**File**: `frontend/src/components/LanguageSwitcher/index.ts`

```typescript
export { LanguageSwitcher } from './LanguageSwitcher';
```

---

### Step 5: Swizzle DocItem Component (10 minutes)

**Run Docusaurus swizzle command**:
```bash
cd frontend
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --eject
```

**File**: `frontend/src/theme/DocItem/Layout/index.tsx` (modify)

```typescript
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import { LanguageSwitcher } from '@/components/LanguageSwitcher';

export default function LayoutWrapper(props) {
  return (
    <>
      <LanguageSwitcher />  {/* INSERT HERE: below title, before content */}
      <Layout {...props} />
    </>
  );
}
```

---

### Step 6: Add RTL Styles (5 minutes)

**File**: `frontend/src/css/rtl.css` (create new)

```css
/* RTL-specific styles for Urdu */
[dir="rtl"] .theme-doc-sidebar {
  right: 0;
  left: auto;
}

[dir="rtl"] .theme-doc-toc {
  left: 0;
  right: auto;
}

[dir="rtl"] .markdown {
  text-align: right;
}

/* Code blocks always LTR */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}
```

**Import in** `frontend/src/css/custom.css`:
```css
@import './rtl.css';
```

---

### Step 7: Build & Test (10 minutes)

**Build translation manifest**:
```bash
node scripts/generate-translation-manifest.js
```

**Start development server**:
```bash
cd frontend
npm run start
```

**Test checklist**:
- [ ] Button appears below chapter title
- [ ] Clicking button switches language
- [ ] URL updates with `?lang=ur` parameter
- [ ] Page navigates to `/ur/docs/...` path
- [ ] RTL layout active (sidebar on right)
- [ ] Code blocks remain LTR
- [ ] Button disabled for untranslated chapters
- [ ] Tooltip shows "Urdu translation coming soon"
- [ ] Language persists after browser refresh

---

## Troubleshooting

### Issue: Button not appearing
**Fix**: Verify swizzled `DocItem/Layout/index.tsx` imports LanguageSwitcher correctly. Check browser console for errors.

### Issue: Translation manifest empty
**Fix**: Ensure Urdu translation files exist in `frontend/i18n/ur/docusaurus-plugin-content-docs/current/`. Re-run manifest generation script.

### Issue: RTL layout not working
**Fix**: Verify `docusaurus.config.ts` has `direction: 'rtl'` in `localeConfigs.ur`. Check browser inspector shows `<html dir="rtl">`.

### Issue: localStorage not persisting
**Fix**: Check browser privacy settings. Try incognito mode to rule out extensions blocking storage.

---

## Testing

Run unit tests:
```bash
cd frontend
npm test -- LanguageSwitcher.test.tsx
```

Run E2E tests:
```bash
npx playwright test language-switching.spec.ts
```

---

## Next Steps

After implementation complete:
1. Run `/sp.tasks` to generate task breakdown for implementation
2. Follow TDD workflow: write tests first, then implementation
3. Verify all success criteria (SC-001 through SC-008) met
4. Create PR following git workflow standards

---

## Key Files Reference

| File | Purpose |
|------|---------|
| `docusaurus.config.ts` | i18n configuration |
| `scripts/generate-translation-manifest.js` | Build-time manifest generation |
| `src/hooks/useLanguagePreference.ts` | Language state management hook |
| `src/components/LanguageSwitcher/` | Main button component |
| `src/theme/DocItem/Layout/index.tsx` | Swizzled component for button injection |
| `src/css/rtl.css` | RTL layout styles |
| `src/translation-manifest.json` | Build output (generated) |

---

**Estimated Total Time**: ~75 minutes (first-time implementation)

**Questions?** Refer to `plan.md`, `research.md`, and `data-model.md` in specs/002-urdu-translation/
