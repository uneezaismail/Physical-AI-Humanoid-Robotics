# Research Findings: Interactive Urdu Translation Button

**Date**: 2025-12-18
**Feature**: 002-urdu-translation
**Phase**: 0 (Research & Resolution of Technical Unknowns)

## Overview

This document consolidates research findings to resolve all "NEEDS CLARIFICATION" items identified in the Technical Context section of plan.md. All decisions are based on Docusaurus official documentation, React best practices, and project constitution principles.

---

## Research Items

### 1. State Management Approach

**Question**: React Context vs custom hooks vs external library for language preference state management?

**Decision**: Custom React Hook (`useLanguagePreference`)

**Rationale**:
- **Simplicity**: Language preference is simple client-side state (current language + localStorage sync). No complex state tree or global updates needed.
- **Performance**: Single source of truth (localStorage + URL parameter). No unnecessary re-renders from Context provider updates.
- **Docusaurus Patterns**: Docusaurus uses hooks for client-side state (e.g., `useColorMode`, `useDocusaurusContext`). Following established patterns.
- **YAGNI Compliance**: No need for Redux/Zustand for 2-language toggle. Custom hook is 50-100 lines vs 200+ for Context provider setup.

**Alternatives Considered**:
1. **React Context**: Rejected - overkill for single preference value. Adds provider wrapper complexity and potential re-render issues.
2. **External Library (Zustand/Redux)**: Rejected - violates YAGNI principle. Adds dependency for 2-state toggle (English/Urdu).
3. **Docusaurus Plugin**: Rejected - i18n already built-in. We're adding UI layer, not replacing i18n system.

**Implementation Pattern**:
```typescript
export function useLanguagePreference() {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');

  // Read from URL parameter → localStorage → default
  useEffect(() => {
    const urlLang = new URLSearchParams(window.location.search).get('lang');
    const storedLang = localStorage.getItem('preferredLanguage');
    const initialLang = urlLang || storedLang || 'en';
    setLanguage(initialLang as 'en' | 'ur');
  }, []);

  // Persist to localStorage + update URL
  const switchLanguage = (newLang: 'en' | 'ur') => {
    setLanguage(newLang);
    localStorage.setItem('preferredLanguage', newLang);
    // Update URL without reload
    const url = new URL(window.location.href);
    url.searchParams.set('lang', newLang);
    window.history.pushState({}, '', url);
  };

  return { language, switchLanguage };
}
```

---

### 2. Docusaurus i18n Integration Approach

**Question**: How to integrate custom language switcher with Docusaurus's built-in i18n system?

**Decision**: Swizzle `DocItem` component + manual locale switching via Docusaurus Router

**Rationale**:
- **Docusaurus i18n Architecture**: Docusaurus builds separate static sites for each locale (`/` for English, `/ur/` for Urdu). Language "switching" is navigation between these paths.
- **Component Swizzling**: Official Docusaurus pattern for customizing theme components. Swizzling `DocItem` allows injecting LanguageSwitcher below title without modifying core theme.
- **Router Integration**: Use `@docusaurus/router`'s `useHistory` to navigate between `/docs/chapter` and `/ur/docs/chapter` paths.

**Implementation Pattern**:
```typescript
import { useHistory, useLocation } from '@docusaurus/router';

function LanguageSwitcher() {
  const history = useHistory();
  const location = useLocation();
  const { language, switchLanguage } = useLanguagePreference();

  const handleToggle = () => {
    const newLang = language === 'en' ? 'ur' : 'en';
    switchLanguage(newLang);

    // Navigate to equivalent page in other locale
    const currentPath = location.pathname;
    const newPath = newLang === 'ur'
      ? `/ur${currentPath}`
      : currentPath.replace('/ur', '');

    history.push(newPath);
  };

  return <button onClick={handleToggle}>...</button>;
}
```

**Alternatives Considered**:
1. **Real-time Content Swapping**: Rejected - violates Docusaurus architecture. Would require fetching markdown, re-parsing, and DOM replacement. Complex and error-prone.
2. **iframe Embedding**: Rejected - accessibility nightmare, SEO issues, breaks navigation.
3. **Custom Plugin**: Rejected - unnecessary when swizzling achieves same result with less code.

**Documentation Reference**:
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [Docusaurus i18n Tutorial](https://docusaurus.io/docs/i18n/tutorial)

---

### 3. RTL Layout Implementation Strategy

**Question**: How to apply RTL (right-to-left) layout for Urdu without breaking English layout?

**Decision**: CSS custom properties + `[dir="rtl"]` attribute selector

**Rationale**:
- **Docusaurus i18n Support**: Docusaurus automatically sets `<html dir="rtl">` for RTL locales when configured in `localeConfigs`.
- **CSS Cascade**: Use attribute selector `[dir="rtl"]` to override styles only when Urdu active. No JavaScript DOM manipulation needed.
- **Specificity**: Attribute selector has higher specificity than class selectors, ensuring RTL overrides apply correctly.

**Implementation Pattern**:

**docusaurus.config.ts**:
```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',  // CRITICAL: Docusaurus sets dir="rtl" automatically
      htmlLang: 'ur-PK',
    },
  },
},
```

**frontend/src/css/rtl.css**:
```css
/* RTL-specific overrides - only apply when dir="rtl" */
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

/* Code blocks remain LTR even in RTL context */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}
```

**Alternatives Considered**:
1. **JavaScript DOM Manipulation**: Rejected - CSS is declarative and faster. No need for runtime DOM queries.
2. **Separate CSS Files**: Rejected - harder to maintain. Single file with `[dir="rtl"]` selector is clearer.
3. **CSS-in-JS**: Rejected - Docusaurus uses CSS modules. Adding styled-components increases bundle size unnecessarily.

**Documentation Reference**:
- [MDN: CSS dir attribute selector](https://developer.mozilla.org/en-US/docs/Web/CSS/:dir)
- [Docusaurus i18n Configuration](https://docusaurus.io/docs/api/docusaurus-config#i18n)

---

### 4. URL Parameter Precedence Logic

**Question**: How to ensure `?lang=ur` parameter overrides stored localStorage preference?

**Decision**: Priority cascade: URL parameter > localStorage > default (English)

**Rationale**:
- **Specification Requirement**: FR-015 mandates URL parameter ensures recipients see sender's language choice "regardless of their stored preference".
- **User Intent**: URL parameter represents explicit sharing intent. Should always take precedence.
- **Fallback Strategy**: Without URL param, use localStorage (returning user). If neither, default to English (new user).

**Implementation Pattern**:
```typescript
function getInitialLanguage(): 'en' | 'ur' {
  // Priority 1: URL parameter (explicit override)
  const urlParams = new URLSearchParams(window.location.search);
  const urlLang = urlParams.get('lang');
  if (urlLang === 'en' || urlLang === 'ur') {
    return urlLang;
  }

  // Priority 2: localStorage (user preference)
  try {
    const storedLang = localStorage.getItem('preferredLanguage');
    if (storedLang === 'en' || storedLang === 'ur') {
      return storedLang;
    }
  } catch (e) {
    // localStorage unavailable (privacy mode, etc.)
    console.warn('localStorage unavailable:', e);
  }

  // Priority 3: Default to English
  return 'en';
}
```

**Edge Cases Handled**:
- Invalid URL parameter values (e.g., `?lang=fr`) → fallback to localStorage/default
- localStorage unavailable (privacy mode) → fallback to default
- URL parameter present but localStorage has different value → URL wins

---

### 5. Translation Availability Detection

**Question**: How to detect if Urdu translation exists for current chapter?

**Decision**: Check existence of translated markdown file path at build time (static JSON manifest)

**Rationale**:
- **Static Site Architecture**: Docusaurus builds static files at build time. Translation availability is known at build time, not runtime.
- **Performance**: Checking file existence client-side (fetch requests) adds latency. Pre-computed manifest is instant.
- **Reliability**: Build-time check prevents 404s. Button disabled if translation unavailable.

**Implementation Pattern**:

**Build-time script** (generates manifest):
```typescript
// scripts/generate-translation-manifest.js
import fs from 'fs';
import path from 'path';

const docsDir = path.resolve(__dirname, '../frontend/docs');
const urduDir = path.resolve(__dirname, '../frontend/i18n/ur/docusaurus-plugin-content-docs/current');

const manifest = {};

function scanDocs(dir, basePath = '') {
  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      scanDocs(filePath, path.join(basePath, file));
    } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
      const relativePath = path.join(basePath, file);
      const urduPath = path.join(urduDir, relativePath);
      manifest[relativePath] = fs.existsSync(urduPath);
    }
  });
}

scanDocs(docsDir);
fs.writeFileSync(
  path.resolve(__dirname, '../frontend/src/translation-manifest.json'),
  JSON.stringify(manifest, null, 2)
);
```

**Runtime usage**:
```typescript
import translationManifest from '@/translation-manifest.json';

function isTranslationAvailable(docPath: string): boolean {
  return translationManifest[docPath] === true;
}
```

**Alternatives Considered**:
1. **Runtime fetch**: Rejected - adds HTTP requests, slower, requires error handling for 404s.
2. **Always assume available**: Rejected - breaks user experience when translation missing (FR-011).
3. **Manually maintained list**: Rejected - error-prone, requires manual updates after each translation.

---

### 6. Logging & Analytics Strategy (Resolves Constitution Check Principle IX)

**Question**: Should language preference changes be logged? How?

**Decision**: Console logging in development; optional analytics integration via custom events

**Rationale**:
- **Development Debugging**: Console logs help debug localStorage issues, URL parameter parsing, and routing.
- **Production Analytics**: Optional integration (Google Analytics, Plausible) via custom events. Not required for MVP.
- **Privacy**: No PII collected. Only language preference ('en' or 'ur') and timestamp.
- **Constitution Compliance**: Structured logging (JSON format) in development, sanitized events in production.

**Implementation Pattern**:
```typescript
function switchLanguage(newLang: 'en' | 'ur') {
  const previousLang = language;

  // Development logging
  if (process.env.NODE_ENV === 'development') {
    console.log('[LanguageSwitcher]', {
      event: 'language_switch',
      from: previousLang,
      to: newLang,
      source: window.location.search.includes('lang=') ? 'url_parameter' : 'button_click',
      timestamp: new Date().toISOString(),
    });
  }

  // Production analytics (optional)
  if (typeof window.gtag !== 'undefined') {
    window.gtag('event', 'language_switch', {
      from_language: previousLang,
      to_language: newLang,
    });
  }

  setLanguage(newLang);
  localStorage.setItem('preferredLanguage', newLang);
  // ... rest of implementation
}
```

**Alternatives Considered**:
1. **No logging**: Rejected - makes debugging difficult.
2. **Always log to console**: Rejected - pollutes production console unnecessarily.
3. **Required analytics**: Rejected - not all users want analytics. Optional is better.

---

## Summary of Decisions

| Unknown | Decision | Impact |
|---------|----------|--------|
| State management | Custom React Hook (`useLanguagePreference`) | Lightweight, follows Docusaurus patterns |
| i18n integration | Swizzle `DocItem` + Docusaurus Router navigation | Official pattern, SEO-friendly |
| RTL layout | CSS `[dir="rtl"]` attribute selector | Automatic via Docusaurus i18n config |
| URL precedence | URL param > localStorage > default | Ensures sharing behavior (FR-015) |
| Translation detection | Build-time manifest (JSON) | Fast, reliable, no runtime overhead |
| Logging strategy | Console (dev) + optional analytics (prod) | Debuggable, privacy-respecting |

**All unknowns resolved.** Ready to proceed to Phase 1 (Design & Contracts).

---

**References**:
- [Docusaurus i18n Official Guide](https://docusaurus.io/docs/i18n/introduction)
- [Docusaurus Swizzling](https://docusaurus.io/docs/swizzling)
- [React Hooks Best Practices](https://react.dev/reference/react)
- [MDN CSS :dir() Selector](https://developer.mozilla.org/en-US/docs/Web/CSS/:dir)
