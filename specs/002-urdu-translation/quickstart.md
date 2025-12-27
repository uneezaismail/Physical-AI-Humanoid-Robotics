# Developer Quickstart: Docusaurus Native Urdu Translation

**Feature**: 002-urdu-translation
**Last Updated**: 2025-12-18

## Overview

This guide provides developers with everything needed to implement Urdu translation using Docusaurus's built-in i18n system. The implementation requires **zero custom code** - only configuration and translated content files.

---

## Prerequisites

- Node.js 18+ installed
- Familiarity with Docusaurus configuration
- Project cloned and dependencies installed (`npm install` in `frontend/`)
- Urdu translations ready (MDX files with translated content)

---

## Architecture at a Glance

```
┌──────────────────────────────────────────────────────────┐
│            User clicks localeDropdown                     │
│            (built-in Docusaurus component)                │
└────────────────────┬─────────────────────────────────────┘
                     │
                     ▼
        ┌────────────────────────┐
        │  Docusaurus Router     │
        │  (built-in)            │
        └────────┬───────────────┘
                 │
       ┌─────────┴──────────┐
       │                    │
       ▼                    ▼
┌──────────────┐    ┌──────────────┐
│  /docs/...   │    │  /ur/docs/   │
│  (English)   │    │  (Urdu+RTL)  │
└──────────────┘    └──────────────┘
```

**Key Points**:
- No custom React components
- No state management or localStorage
- No custom routing logic
- Pure configuration + content files

---

## Implementation Steps

### Step 1: Configure Docusaurus i18n (2 minutes)

**File**: `frontend/docusaurus.config.ts`

Add the following configuration:

```typescript
// frontend/docusaurus.config.ts
import type { Config } from "@docusaurus/types";

const config: Config = {
  // ... existing config

  // CRITICAL for Vercel deployment
  trailingSlash: false,

  // i18n Configuration
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
        direction: "rtl",  // Enables automatic RTL layout
        htmlLang: "ur-PK",
      },
    },
  },

  // ... existing themeConfig
  themeConfig: {
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Textbook",
        },
        {
          type: "localeDropdown",  // ADD THIS: Built-in language selector
          position: "right",
        },
        {
          type: "custom-auth-button",
          position: "right",
        },
        // ... other navbar items
      ],
    },
    // ... rest of themeConfig
  },
};

export default config;
```

**What This Does**:
- Adds Urdu as a supported locale
- Configures RTL (right-to-left) layout for Urdu
- Adds language dropdown to navbar
- Sets `trailingSlash: false` for Vercel compatibility

**Verify**: Run `npm run start` and check that language dropdown appears in navbar (even if no translations exist yet).

---

### Step 2: Create Translation Directory Structure (1 minute)

Create the directory for Urdu translations:

```bash
cd frontend
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
```

**Directory Structure**:
```
frontend/i18n/
└── ur/
    └── docusaurus-plugin-content-docs/
        ├── current.json               # Sidebar labels (create next)
        └── current/                   # Chapter translations (create next)
```

---

### Step 3: Add Sidebar Label Translations (5 minutes)

**File**: `frontend/i18n/ur/docusaurus-plugin-content-docs/current.json`

Create this file with Urdu sidebar labels:

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
  },
  "chapter-03-physical-ai-architecture": {
    "message": "باب 3: فزیکل AI آرکیٹیکچر",
    "description": "Chapter 3 sidebar label"
  },
  "part-2-robotic-nervous-system": {
    "message": "حصہ II: روبوٹک اعصابی نظام",
    "description": "Part 2 category label"
  },
  "chapter-04-ros2-architecture": {
    "message": "باب 4: ROS 2 آرکیٹیکچر",
    "description": "Chapter 4 sidebar label"
  },
  "chapter-05-publisher-subscriber": {
    "message": "باب 5: پبلشر-سبسکرائبر",
    "description": "Chapter 5 sidebar label"
  },
  "chapter-06-services-actions": {
    "message": "باب 6: سروسز اور ایکشنز",
    "description": "Chapter 6 sidebar label"
  },
  "chapter-07-parameters-launch": {
    "message": "باب 7: پیرامیٹرز اور لانچ",
    "description": "Chapter 7 sidebar label"
  },
  "chapter-08-sensor-integration": {
    "message": "باب 8: سینسر انضمام",
    "description": "Chapter 8 sidebar label"
  },
  "chapter-09-gazebo-simulation": {
    "message": "باب 9: Gazebo سمولیشن",
    "description": "Chapter 9 sidebar label"
  }
}
```

**What This Does**: Translates sidebar navigation labels (chapter titles, category names) to Urdu.

---

### Step 4: Add Translated Chapter Content (Varies)

**Directory**: `frontend/i18n/ur/docusaurus-plugin-content-docs/current/`

Mirror the structure of `frontend/docs/` and add translated MDX files.

**Example**: Translating Chapter 1

**English Source** (`frontend/docs/part-1-foundations-lab/chapter-01-embodied-ai.mdx`):
```mdx
---
title: "Chapter 1: Embodied Intelligence"
sidebar_position: 1
---

# Chapter 1: Embodied Intelligence

Introduction to embodied AI...

## What is Physical AI?

Physical AI refers to...

```python
# Example code (keep in English)
import rclpy
```
```

**Urdu Translation** (`frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-1-foundations-lab/chapter-01-embodied-ai.mdx`):
```mdx
---
title: "باب 1: جسمانی ذہانت"
sidebar_position: 1
---

# باب 1: جسمانی ذہانت

جسمانی AI کا تعارف...

## فزیکل AI کیا ہے؟

فزیکل AI سے مراد...

```python
# Example code (keep in English - do NOT translate)
import rclpy
```
```

**Important Rules**:
1. **Keep Directory Structure Identical**: Urdu files must mirror English structure exactly
2. **Keep Filenames Identical**: Same filename as English version
3. **Translate Frontmatter**: Translate `title`, but keep `sidebar_position` value same
4. **Translate Prose**: Translate paragraphs, headings, lists
5. **DO NOT Translate Code**: Code blocks remain in English
6. **DO NOT Translate Paths**: File paths, URLs, command examples stay English

---

### Step 5: Build and Test Locally (5 minutes)

**Build Both Locales**:
```bash
cd frontend
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files for "en" locale.
[SUCCESS] Generated static files for "ur" locale.

Build output:
  ├── build/          (English)
  └── build/ur/       (Urdu)
```

**Preview Locally**:
```bash
npx http-server build -p 3000
```

**Test Checklist**:
1. Visit `http://localhost:3000/docs/part-1-foundations-lab/chapter-01-embodied-ai`
2. Click language dropdown (top-right navbar)
3. Select "اردو" from dropdown
4. Verify navigation to `/ur/docs/...`
5. Verify content is in Urdu
6. Verify RTL layout (sidebar on right, text right-aligned)
7. Verify code blocks remain in English (LTR)
8. Click "English" in dropdown to switch back
9. Verify navigation to `/docs/...` (English version)

---

### Step 6: Configure Vercel Deployment (2 minutes)

**File**: `frontend/vercel.json` (should contain ONLY API rewrites):

```json
{
  "rewrites": [
    {
      "source": "/api/auth/:match*",
      "destination": "https://your-backend.railway.app/api/auth/:match*"
    },
    {
      "source": "/api/chat/:match*",
      "destination": "https://your-backend.railway.app/api/chat/:match*"
    }
  ]
}
```

**Vercel Dashboard Settings** (configure manually):

1. Go to Vercel project settings
2. Navigate to "General" → "Build & Development Settings"
3. Set **Framework Preset**: `Docusaurus (v2+)`
4. Set **Root Directory**: `frontend`
5. Leave **Build Command** and **Output Directory** as auto-detected
6. Save settings

**Why This Matters**:
- Docusaurus project lives in `frontend/` subdirectory
- Vercel needs to know where to run `npm install` and `npm run build`
- Framework Preset enables auto-detection of build commands

---

### Step 7: Deploy and Verify (5 minutes)

**Commit and Push**:
```bash
git add frontend/docusaurus.config.ts frontend/i18n/ frontend/vercel.json
git commit -m "feat: Add Urdu translation with native Docusaurus i18n"
git push origin main
```

**Wait for Vercel Build** (check deployment logs):
- Build should succeed for both locales
- Both `build/` and `build/ur/` directories should be generated
- No 404 errors

**Production Test Checklist**:
1. Visit production URL (e.g., `https://your-site.vercel.app`)
2. Navigate to `/docs/part-1-foundations-lab/chapter-01-embodied-ai`
3. Click language dropdown → Select "اردو"
4. Verify Urdu content loads at `/ur/docs/...`
5. Verify RTL layout
6. Test direct URL access: Share `/ur/docs/...` link with someone, confirm it works
7. Test on mobile device
8. Test in different browsers (Chrome, Firefox, Safari)

---

## Troubleshooting

### Issue: Language dropdown doesn't appear

**Cause**: `localeDropdown` not added to navbar
**Fix**: Check `docusaurus.config.ts` → `themeConfig.navbar.items` → Ensure `{type: "localeDropdown", position: "right"}` exists

---

### Issue: 404 on `/ur/docs/*` URLs in production

**Cause**: Vercel not building Urdu locale
**Fix**:
1. Check Vercel Dashboard → Settings → Framework Preset (should be "Docusaurus (v2+)")
2. Check Vercel Dashboard → Settings → Root Directory (should be "frontend")
3. Redeploy project

---

### Issue: Urdu content shows English

**Cause**: Translation file missing or path mismatch
**Fix**:
1. Verify file exists at `frontend/i18n/ur/docusaurus-plugin-content-docs/current/<path-to-chapter>.mdx`
2. Verify directory structure mirrors `frontend/docs/` exactly
3. Verify filename matches English version exactly
4. Check build logs for fallback warnings

---

### Issue: RTL layout not working

**Cause**: Locale direction not set to RTL
**Fix**: Check `docusaurus.config.ts` → `i18n.localeConfigs.ur.direction` is `"rtl"`

---

### Issue: Code blocks are in Urdu (wrong!)

**Cause**: Translator accidentally translated code
**Fix**: Edit Urdu MDX file → Restore English code in code blocks (````python` ... ```)

---

## Summary

**What You Did**:
1. ✅ Added `i18n` config to `docusaurus.config.ts` (5 lines)
2. ✅ Added `localeDropdown` to navbar (4 lines)
3. ✅ Created `i18n/ur/.../current.json` with sidebar labels (1 file)
4. ✅ Added translated MDX files (9 chapters)
5. ✅ Configured Vercel dashboard (2 settings)

**What You Didn't Need**:
- ❌ No custom React components
- ❌ No state management hooks
- ❌ No localStorage logic
- ❌ No custom routing
- ❌ No manual RTL CSS
- ❌ No build scripts

**Total Implementation**: ~30 minutes configuration + translation time

---

## Adding More Languages (Future)

To add another language (e.g., Arabic):

1. Add locale to `docusaurus.config.ts`:
   ```typescript
   locales: ["en", "ur", "ar"],
   localeConfigs: {
     ar: {
       label: "العربية",
       direction: "rtl",
       htmlLang: "ar-SA",
     },
   }
   ```

2. Create directory:
   ```bash
   mkdir -p i18n/ar/docusaurus-plugin-content-docs/current
   ```

3. Add translated content in `i18n/ar/.../current/`
4. Build and deploy

That's it! Docusaurus handles the rest.

---

## References

- [Docusaurus i18n Tutorial](https://docusaurus.io/docs/i18n/tutorial)
- [Docusaurus i18n API](https://docusaurus.io/docs/api/docusaurus-config#i18n)
- [Vercel Docusaurus Guide](https://vercel.com/guides/deploying-docusaurus-with-vercel)
