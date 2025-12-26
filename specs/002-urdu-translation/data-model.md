# Data Model: Docusaurus Native Urdu Translation

**Date**: 2025-12-18
**Feature**: 002-urdu-translation
**Phase**: 1 (Design & Contracts)

## Overview

This document defines the data structures and file organization for the Docusaurus Native Urdu Translation feature. Since this implementation uses Docusaurus's built-in i18n system, there are **no custom data models, state management, or client-side storage**. The "data model" consists entirely of file structure and configuration.

---

## Entity Definitions

### 1. Locale Configuration

**Description**: Docusaurus locale settings that define supported languages and their properties.

**Storage Location**: `frontend/docusaurus.config.ts` (configuration file)

**TypeScript Interface** (from `@docusaurus/types`):
```typescript
interface I18nConfig {
  /** Default locale used when no locale prefix in URL */
  defaultLocale: string;

  /** List of all supported locales */
  locales: string[];

  /** Directory where i18n translation files are stored */
  path: string;

  /** Configuration for each locale */
  localeConfigs: {
    [locale: string]: {
      /** Display label in language selector */
      label: string;

      /** Text direction: 'ltr' or 'rtl' */
      direction: 'ltr' | 'rtl';

      /** HTML lang attribute value */
      htmlLang: string;

      /** Optional: Calendar system */
      calendar?: string;

      /** Optional: Path prefix for this locale */
      path?: string;
    };
  };
}
```

**Implementation Example**:
```typescript
// frontend/docusaurus.config.ts
i18n: {
  defaultLocale: "en",
  locales: ["en", "ur"],
  path: "i18n",
  localeConfigs: {
    en: {
      label: "English",
      direction: "ltr",
      htmlLang: "en-US",
    },
    ur: {
      label: "اردو",
      direction: "rtl",
      htmlLang: "ur-PK",
    },
  },
}
```

---

### 2. Translation File Structure

**Description**: Organization of translated content files for each locale.

**Storage Location**: `frontend/i18n/<locale>/docusaurus-plugin-content-docs/`

**Directory Schema**:
```
frontend/i18n/
└── ur/                              # Locale code
    └── docusaurus-plugin-content-docs/
        ├── current.json             # Sidebar label translations
        └── current/                 # Translated markdown files
            ├── part-1-foundations-lab/
            │   ├── chapter-01-embodied-ai.mdx
            │   ├── chapter-02-hardware-setup.mdx
            │   └── chapter-03-physical-ai-architecture.mdx
            └── part-2-robotic-nervous-system/
                ├── chapter-04-ros2-architecture.mdx
                ├── chapter-05-publisher-subscriber.mdx
                ├── chapter-06-services-actions.mdx
                ├── chapter-07-parameters-launch.mdx
                ├── chapter-08-sensor-integration.mdx
                └── chapter-09-gazebo-simulation.mdx
```

**Validation Rules**:
- Directory structure MUST mirror `frontend/docs/` exactly
- MDX files MUST have identical filenames to English versions
- Frontmatter fields (title, sidebar_position, etc.) can be translated
- Code blocks MUST remain in English (not translated)

---

### 3. Sidebar Label Translations

**Description**: JSON file containing translations for sidebar navigation elements.

**Storage Location**: `frontend/i18n/ur/docusaurus-plugin-content-docs/current.json`

**JSON Schema**:
```typescript
interface SidebarTranslations {
  [key: string]: {
    /** Translated label text */
    message: string;

    /** English description for translators */
    description?: string;
  };
}
```

**Implementation Example**:
```json
{
  "tutorialSidebar": {
    "message": "رہنمائی سائڈبار",
    "description": "The label for the docs sidebar"
  },
  "part-1-foundations-lab": {
    "message": "حصہ I: بنیادیں اور لیبارٹری",
    "description": "Part 1 category label"
  },
  "chapter-01-embodied-ai": {
    "message": "باب 1: جسمانی ذہانت",
    "description": "Chapter 1 sidebar label"
  },
  "chapter-02-hardware-setup": {
    "message": "باب 2: ہارڈویئر سیٹ اپ",
    "description": "Chapter 2 sidebar label"
  }
}
```

---

### 4. URL Structure (Runtime Data Model)

**Description**: How Docusaurus maps URLs to locale-specific content at runtime.

**Data Flow**:
```
User Request: /docs/part-1-foundations-lab/chapter-01-embodied-ai
    ↓
Docusaurus Router: Detect locale from URL (no prefix = default locale 'en')
    ↓
Content Source: frontend/docs/part-1-foundations-lab/chapter-01-embodied-ai.mdx
    ↓
Render: HTML with <html lang="en-US" dir="ltr">

---

User Request: /ur/docs/part-1-foundations-lab/chapter-01-embodied-ai
    ↓
Docusaurus Router: Detect locale from URL (prefix '/ur' = Urdu locale)
    ↓
Content Source: frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-1-foundations-lab/chapter-01-embodied-ai.mdx
    ↓
Fallback (if file missing): frontend/docs/part-1-foundations-lab/chapter-01-embodied-ai.mdx (English content with Urdu URL)
    ↓
Render: HTML with <html lang="ur-PK" dir="rtl">
```

**URL Mapping Table**:

| English URL | Urdu URL | Content Source |
|-------------|----------|----------------|
| `/` | `/ur/` | Homepage (English only, no translation) |
| `/docs/intro` | `/ur/docs/intro` | Docs intro (Urdu if translated, else English) |
| `/docs/part-1/.../chapter-01` | `/ur/docs/part-1/.../chapter-01` | Chapter 1 (Urdu translation) |
| `/auth` | `/ur/auth` | Auth page (English only, no translation) |

---

### 5. Build Output Structure

**Description**: Static HTML files generated during build process for each locale.

**Storage Location**: `frontend/build/` and `frontend/build/ur/`

**Build Output Schema**:
```
frontend/build/
├── index.html                     # Homepage (English)
├── docs/
│   ├── intro/
│   │   └── index.html            # English docs intro
│   ├── part-1-foundations-lab/
│   │   ├── chapter-01-embodied-ai/
│   │   │   └── index.html        # English Chapter 1
│   │   └── ...
│   └── ...
├── auth/
│   └── index.html                # Auth page (English)
├── sitemap.xml                   # English sitemap
└── ur/                           # Urdu locale directory
    ├── index.html                # Homepage (same as English)
    ├── docs/
    │   ├── intro/
    │   │   └── index.html        # Urdu docs intro
    │   ├── part-1-foundations-lab/
    │   │   ├── chapter-01-embodied-ai/
    │   │   │   └── index.html    # Urdu Chapter 1 (RTL layout)
    │   │   └── ...
    │   └── ...
    ├── auth/
    │   └── index.html            # Auth page (same as English)
    └── sitemap.xml               # Urdu sitemap
```

**HTML Attributes**:
- English pages: `<html lang="en-US" dir="ltr">`
- Urdu pages: `<html lang="ur-PK" dir="rtl">`
- All pages include `<link rel="alternate" hreflang="..." href="...">` tags for SEO

---

## Data Relationships

### Content → Locale Mapping

```
┌─────────────────────────┐
│   English Content       │
│  (source of truth)      │
│                         │
│  docs/                  │
│  ├── chapter-01.mdx     │
│  └── chapter-02.mdx     │
└───────────┬─────────────┘
            │
            │ Translation Process
            │ (manual, external to system)
            ▼
┌─────────────────────────┐
│   Urdu Translation      │
│  (derived content)      │
│                         │
│  i18n/ur/.../current/   │
│  ├── chapter-01.mdx     │
│  └── chapter-02.mdx     │
└─────────────────────────┘
```

### URL → Content Resolution

```
User Request: /ur/docs/chapter-01
        │
        ▼
┌───────────────────────────┐
│  Docusaurus Router        │
│  1. Parse locale: 'ur'    │
│  2. Parse path: /docs/... │
└───────┬───────────────────┘
        │
        ▼
┌───────────────────────────────────────┐
│  Content Plugin (docs)                │
│  1. Check: i18n/ur/.../chapter-01.mdx │
│  2. If found: Use Urdu content        │
│  3. If missing: Fallback to English   │
└───────┬───────────────────────────────┘
        │
        ▼
┌───────────────────────────┐
│  Theme Renderer           │
│  1. Apply locale config   │
│  2. Set dir="rtl"         │
│  3. Set lang="ur-PK"      │
│  4. Render Urdu sidebar   │
└───────────────────────────┘
```

---

## State Management

**Question**: Where is application state stored?

**Answer**: **No application state.** This is a pure static site with no client-side state management.

**Rationale**:
- Language selection is URL-based (state is the URL itself)
- No localStorage, sessionStorage, or cookies
- No React Context, Redux, or state libraries
- Docusaurus handles all routing internally

**What Happens When User Switches Language**:
1. User clicks localeDropdown → Selects "اردو"
2. Docusaurus built-in component triggers navigation
3. Browser navigates to `/ur/docs/...` URL (standard page navigation)
4. Docusaurus serves pre-built static HTML for Urdu locale
5. No state updates, no re-renders, no client-side logic

---

## Data Validation & Constraints

### Translation File Validation

**Required Validations** (enforced during build):
- All translated MDX files must have valid frontmatter
- Sidebar label keys in `current.json` must match sidebar IDs
- No circular references in sidebar structure
- Urdu content must use UTF-8 encoding

**Optional Validations** (recommended):
- Check for untranslated code blocks (should remain English)
- Verify RTL Unicode characters in Urdu text
- Validate that English and Urdu files have same structure (headings, sections)

### Build-Time Checks

**Docusaurus Automatic Checks**:
- Warns if sidebar label translation missing (falls back to English)
- Errors if MDX syntax invalid
- Warns if internal links broken (links to untranslated pages)

---

## Summary: Simplified Data Model

Unlike traditional web applications, this feature has **no runtime data model**. All "data" is:

1. **Configuration** (docusaurus.config.ts)
2. **Files** (MDX content, sidebar JSON)
3. **URLs** (path-based locale selection)

**No Need For**:
- TypeScript interfaces for state
- Data validation at runtime
- Client-side storage schemas
- API contracts or data fetching

This simplicity is the core advantage of using Docusaurus native i18n: **configuration-only, zero custom data management**.

---

## References

- [Docusaurus i18n File Structure](https://docusaurus.io/docs/i18n/tutorial#translate-your-site)
- [Docusaurus Docs Plugin i18n](https://docusaurus.io/docs/api/plugins/@docusaurus/plugin-content-docs#i18n)
