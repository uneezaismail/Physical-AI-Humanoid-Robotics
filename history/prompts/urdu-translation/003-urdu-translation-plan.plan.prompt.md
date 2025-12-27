---
id: 003
title: Interactive Urdu Translation Button Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: urdu-translation
branch: 002-urdu-translation
user: uneezaismail
command: /sp.plan
labels: ["planning", "architecture", "i18n", "docusaurus", "react", "typescript", "design"]
links:
  spec: specs/002-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-urdu-translation/plan.md
  - specs/002-urdu-translation/research.md
  - specs/002-urdu-translation/data-model.md
  - specs/002-urdu-translation/quickstart.md
  - specs/002-urdu-translation/contracts/LanguageSwitcher.contract.ts
  - CLAUDE.md (context updated)
tests:
  - Constitution Check (10/10 principles passed)
---

## Prompt

/sp.plan

## Response snapshot

Created comprehensive implementation plan for Interactive Urdu Translation Button feature following Spec-Driven Development workflow:

**Phase 0 - Research & Resolution:**
- Resolved 6 technical unknowns from Technical Context
- **Key Decisions:**
  1. State Management: Custom React Hook (`useLanguagePreference`) over Context/Redux
  2. i18n Integration: Swizzle DocItem + Docusaurus Router navigation
  3. RTL Layout: CSS `[dir="rtl"]` attribute selector (Docusaurus built-in)
  4. URL Precedence: URL param > localStorage > default English
  5. Translation Detection: Build-time manifest (JSON) for performance
  6. Logging: Console (dev) + optional Google Analytics (prod)

**Phase 1 - Design & Contracts:**
- **Data Model**: 4 entities defined (LanguagePreference, TranslationButtonState, TranslationManifest, URLParameter)
- **API Contract**: TypeScript interfaces for all components, hooks, and utilities
- **Quickstart Guide**: 7-step implementation guide (~75 minutes estimated)

**Architecture Summary:**
- **Technology**: TypeScript 5.x, React 18+, Docusaurus 3.x
- **Storage**: Browser localStorage (client-side only)
- **Components**: LanguageSwitcher (React), useLanguagePreference (custom hook)
- **Integration**: Swizzled DocItem/Layout component for button injection
- **RTL Support**: Automatic via Docusaurus `direction: 'rtl'` config + CSS overrides

**Constitution Check:**
- ✅ All 10 principles PASSED
- Principle IX (Observability) resolved via research phase
- No YAGNI violations
- No complexity tracking needed

**Project Structure:**
- Frontend-only (static site)
- 3 new files: LanguageSwitcher component + hook + utils
- 1 swizzled component: DocItem/Layout
- 1 build script: generate-translation-manifest.js
- RTL styles: rtl.css

**Key Implementation Patterns:**
1. URL parameter precedence for sharing (FR-015)
2. Pre-translated content files (no runtime translation)
3. Client-side routing between `/docs/` and `/ur/docs/` paths
4. Build-time translation availability detection
5. localStorage persistence with error handling

**Performance Targets:**
- Language switch: < 2 seconds (SC-001)
- Preference detection: < 100ms (SC-008)
- No page reloads (FR-014)
- Cross-browser compatible (SC-006)

## Outcome

- ✅ Impact: Complete implementation plan ready for task breakdown. All technical unknowns resolved. Architecture aligns with Docusaurus patterns and project constitution.
- 🧪 Tests: Constitution Check passed 10/10 principles
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, LanguageSwitcher.contract.ts
- 🔁 Next prompts: `/sp.tasks` to generate detailed task breakdown with test cases
- 🧠 Reflection: Planning phase successfully bridged specification to implementation. Research phase critical for resolving state management approach. Docusaurus swizzling pattern provides clean integration point. TypeScript contracts ensure type safety throughout implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None. All technical decisions well-researched with documented alternatives.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (10/10)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (research) + Phase 1 (design)
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to break down implementation into testable tasks following TDD workflow
