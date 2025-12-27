# Research Findings: Docusaurus Native Urdu Translation

**Date**: 2025-12-18
**Feature**: 002-urdu-translation
**Phase**: 0 (Research & Resolution of Technical Unknowns)

## Overview

This document consolidates research findings for implementing multilingual support using Docusaurus's built-in internationalization (i18n) system. All decisions are based on Docusaurus official documentation, web standards, and project constitution principles.

---

## Research Items

### 1. i18n System Selection

**Question**: Custom translation system vs Docusaurus native i18n?

**Decision**: Docusaurus Native i18n

**Rationale**:
- **Official Support**: Docusaurus provides a complete, well-documented i18n system that is actively maintained by the core team.
- **Zero Custom Code**: No custom React components, hooks, or state management required. Pure configuration.
- **SEO Optimization**: Automatic generation of `hreflang` tags, separate sitemaps per locale, and proper `lang` attributes.
- **RTL Support**: Built-in RTL layout via locale configuration (`direction: "rtl"`). Docusaurus theme automatically handles text direction, sidebar mirroring, and layout transformations.
- **URL-Based Language Selection**: Standard web pattern (`/docs/...` for English, `/ur/docs/...` for Urdu). Shareable, bookmarkable, and predictable.
- **YAGNI Compliance**: Simplest possible solution. No over-engineering.

**Alternatives Considered**:
1. **Custom React Components with localStorage**: Rejected - Over-engineered, breaks URL sharing, non-standard, higher maintenance burden.
2. **Runtime Translation API (e.g., Google Translate)**: Rejected - Slow, inaccurate for technical content, requires backend, violates accuracy principles.
3. **Docusaurus Plugin Development**: Rejected - i18n already built-in, would duplicate existing functionality.

**Implementation Pattern**:
```typescript
// frontend/docusaurus.config.ts
const config: Config = {
  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
    localeConfigs: {
      en: {
        label: "English",
        direction: "ltr",
        htmlLang: "en-US",
      },
      ur: {
        label: "اردو",
        direction: "rtl",  // Automatic RTL layout
        htmlLang: "ur-PK",
      },
    },
  },
};
```

---

### 2. Language Selection UI

**Question**: Custom language switcher component vs built-in localeDropdown?

**Decision**: Built-in `localeDropdown` Navbar Item

**Rationale**:
- **Zero Code Required**: Configuration-only (add navbar item in `docusaurus.config.ts`).
- **Maintained by Docusaurus**: Automatic updates, bug fixes, and accessibility improvements.
- **Accessible by Default**: Keyboard navigation, screen reader support, ARIA labels handled automatically.
- **Consistent UX**: Follows Docusaurus ecosystem patterns, familiar to users of other Docusaurus sites.
- **Works Everywhere**: Appears on all page types (docs, homepage, auth) without custom integration logic.

**Alternatives Considered**:
1. **Custom LanguageSwitcher Component**: Rejected - Unnecessary code, accessibility burden, maintenance overhead, violates YAGNI.
2. **Language Buttons in Footer**: Rejected - Poor discoverability, not standard pattern.
3. **Browser Language Detection + Auto-Redirect**: Rejected - Breaks URL sharing, confusing UX, non-standard.

**Implementation Pattern**:
```typescript
// frontend/docusaurus.config.ts
themeConfig: {
  navbar: {
    items: [
      {
        type: "localeDropdown",  // Built-in component
        position: "right",
      },
    ],
  },
}
```

---

### 3. Translation Scope

**Question**: Translate entire site or docs-only?

**Decision**: Docs-Only Translation

**Rationale**:
- **Educational Focus**: Textbook chapters benefit most from translation. Homepage, auth, and UI elements are minimal and universally understood.
- **Reduced Maintenance**: Translating only educational content (9 chapters) vs entire site (homepage, auth, navbar, footer, marketing pages).
- **Security Simplicity**: Auth system remains in single language, avoiding complexities with error messages, validation, email templates.
- **Clear User Expectation**: Textbook is multilingual, platform is English. Reduces cognitive load.

**Implementation**:
- **Translate**: Only files in `i18n/ur/docusaurus-plugin-content-docs/` (chapters + sidebar labels)
- **Do NOT Translate**: `i18n/ur/docusaurus-theme-classic/navbar.json`, `footer.json`, `code.json`, or any non-docs content

**File Structure**:
```
i18n/ur/
  └── docusaurus-plugin-content-docs/
      ├── current.json              # Sidebar labels only
      └── current/
          ├── part-1-foundations-lab/
          │   ├── chapter-01-embodied-ai.mdx
          │   └── ...
          └── part-2-robotic-nervous-system/
              └── ...
```

---

### 4. Language Switching Behavior

**Question**: Auto-redirect based on preference vs URL-based navigation?

**Decision**: No Auto-Redirect, URL is Source of Truth

**Rationale**:
- **Predictable Behavior**: Users land on the URL they requested (shared links work correctly).
- **Web Standards**: URL-based language selection is standard pattern (e.g., Wikipedia, Google Docs).
- **Shareable Links**: `/ur/docs/...` URLs work for all recipients, regardless of their preferences.
- **No localStorage Required**: Simpler, no privacy concerns, works in incognito mode.
- **Respects User Intent**: If user clicks English link, show English (don't override with stored preference).

**Docusaurus Behavior**:
- User clicks localeDropdown → Docusaurus navigates to equivalent page in new locale
- `/docs/chapter-01` → User selects "اردو" → Navigate to `/ur/docs/chapter-01`
- No redirects, no localStorage checks, pure URL-based routing

---

### 5. RTL Layout Implementation

**Question**: Custom CSS for RTL vs Docusaurus built-in support?

**Decision**: Docusaurus Built-in RTL Support

**Rationale**:
- **Automatic**: When `direction: "rtl"` is set in locale config, Docusaurus theme automatically:
  - Adds `dir="rtl"` to `<html>` element
  - Applies RTL CSS transformations (text-align, padding, margins, flex-direction)
  - Mirrors layout components (sidebar moves to right, navbar items flip)
  - Preserves LTR for code blocks (keeps code readable)
- **No Custom CSS**: Docusaurus theme CSS already includes RTL support for all components
- **Tested & Maintained**: RTL support is part of Docusaurus core, used by many Arabic/Hebrew/Urdu sites

**What Happens**:
```html
<!-- English page -->
<html dir="ltr" lang="en-US">
  <body>
    <nav>...</nav>
    <aside class="sidebar-left">...</aside>  <!-- Left side -->
    <main>Text flows left-to-right</main>
  </body>
</html>

<!-- Urdu page (automatic transformation) -->
<html dir="rtl" lang="ur-PK">
  <body>
    <nav>...</nav>
    <aside class="sidebar-right">...</aside>  <!-- Right side -->
    <main>متن دائیں سے بائیں</main>  <!-- Right-to-left text -->
  </body>
</html>
```

---

### 6. Deployment Configuration (Vercel)

**Question**: How to deploy Docusaurus i18n to Vercel with subdirectory project structure?

**Decision**: Configure Root Directory in Vercel Dashboard

**Research Findings**:
- Docusaurus projects in subdirectories require explicit Vercel configuration
- **Framework Preset**: Must be set to "Docusaurus (v2+)" for auto-detection of build commands
- **Root Directory**: Must be set to `frontend` (where package.json lives)
- **Build Output**: Docusaurus automatically generates `build/` (English) and `build/ur/` (Urdu)
- **No Manual Build Commands**: Vercel auto-detects `npm run build` when Framework Preset is configured

**Vercel Dashboard Settings**:
```
Framework: Docusaurus (v2+)
Root Directory: frontend
Build Command: (auto-detected)
Output Directory: (auto-detected as 'build')
```

**Why Not `vercel.json` Build Commands**:
- Hardcoded paths in `vercel.json` break when Vercel's working directory changes
- Framework Preset auto-detection is more reliable and maintained by Vercel
- `vercel.json` should only contain API rewrites, not build configuration

---

## Decision Summary Table

| Decision Point | Chosen Approach | Rejected Alternatives |
|----------------|----------------|----------------------|
| i18n System | Docusaurus Native | Custom components, Translation API |
| Language UI | localeDropdown | Custom switcher, Footer links |
| Translation Scope | Docs-only | Full site translation |
| Language Switching | URL-based navigation | Auto-redirect, localStorage preference |
| RTL Layout | Docusaurus automatic | Custom CSS, Manual transformations |
| Deployment | Vercel Dashboard config | vercel.json build commands |

---

## Technical Trade-offs

### Accepted Trade-offs

1. **Manual Translation Required**: Pre-translated MDX files needed (benefit: accuracy, control over quality).
2. **Page Navigation for Language Switch**: Switching languages requires page reload (benefit: standard web behavior, SEO-friendly, shareable URLs).
3. **Docs-Only Translation**: Homepage/auth remain English (benefit: reduced maintenance, simpler auth UX).

### Rejected Complexity

1. **No Custom State Management**: No React Context, Redux, Zustand (avoided 200+ lines of boilerplate).
2. **No Client-Side Routing Logic**: No custom URL manipulation, localStorage sync, or preference detection (avoided 100+ lines of edge case handling).
3. **No Custom RTL CSS**: No manual text-align, flexbox reversals, or layout calculations (avoided 50+ lines of fragile CSS).

---

## Constitution Compliance

All decisions align with project constitution:

- **Principle I (Educational Excellence)**: Urdu translation expands accessibility without compromising content quality.
- **Principle II (Technical Accuracy)**: Manual translations ensure technical terms are accurately conveyed.
- **Principle III (Spec-Driven Development)**: Research phase completed before design/implementation.
- **Principle X (YAGNI)**: Zero custom code. Pure configuration-based implementation.

---

## References

- [Docusaurus i18n Tutorial](https://docusaurus.io/docs/i18n/tutorial)
- [Docusaurus i18n Configuration](https://docusaurus.io/docs/api/docusaurus-config#i18n)
- [Vercel Docusaurus Deployment](https://vercel.com/guides/deploying-docusaurus-with-vercel)
- [W3C: Structural Markup and Right-to-Left Text in HTML](https://www.w3.org/International/questions/qa-html-dir)
