/**
 * TypeScript types for i18n (Internationalization)
 *
 * This file contains type definitions for language preference management
 * and translation functionality. Based on contracts/LanguageSwitcher.contract.ts
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Supported language codes following ISO 639-1 standard
 * - 'en': English
 * - 'ur': Urdu
 */
export type LanguageCode = 'en' | 'ur';

/**
 * Text direction for layout
 * - 'ltr': Left-to-Right (English)
 * - 'rtl': Right-to-Left (Urdu, Arabic, Hebrew)
 */
export type TextDirection = 'ltr' | 'rtl';

// ============================================================================
// Component Props
// ============================================================================

/**
 * Props for the LanguageSwitcher component
 */
export interface LanguageSwitcherProps {
  /**
   * Optional CSS class name for styling
   */
  className?: string;

  /**
   * Whether to show the button as an icon only (for mobile)
   * @default false
   */
  iconOnly?: boolean;

  /**
   * Optional callback when language changes
   * Useful for analytics or custom behavior
   */
  onLanguageChange?: (newLanguage: LanguageCode) => void;
}

// ============================================================================
// State Interfaces
// ============================================================================

/**
 * Language preference state managed by useLanguagePreference hook
 */
export interface LanguagePreferenceState {
  /**
   * Currently active language
   */
  language: LanguageCode;

  /**
   * Whether the language preference is being loaded/initialized
   */
  isLoading: boolean;

  /**
   * Error state if preference loading/saving fails
   */
  error: Error | null;
}

/**
 * Translation button UI state
 */
export interface TranslationButtonState {
  /**
   * Current language being displayed
   */
  currentLanguage: LanguageCode;

  /**
   * Whether Urdu translation exists for current chapter
   */
  translationAvailable: boolean;

  /**
   * Loading state during language switch navigation
   */
  isLoading: boolean;

  /**
   * Button disabled state (true if translation unavailable)
   */
  disabled: boolean;

  /**
   * Button label text
   */
  label: string;

  /**
   * Optional tooltip text (shown when disabled)
   */
  tooltip?: string;
}

// ============================================================================
// Hook Return Types
// ============================================================================

/**
 * Return type for useLanguagePreference custom hook
 */
export interface UseLanguagePreferenceReturn {
  /**
   * Current language state
   */
  language: LanguageCode;

  /**
   * Function to switch to a different language
   * @param newLanguage - Language code to switch to
   */
  switchLanguage: (newLanguage: LanguageCode) => void;

  /**
   * Whether preference is currently being loaded
   */
  isLoading: boolean;

  /**
   * Error state if any operation failed
   */
  error: Error | null;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Translation manifest mapping doc paths to availability
 * Generated at build time by scripts/generate-translation-manifest.js
 */
export interface TranslationManifest {
  /**
   * Maps relative doc path to boolean indicating if Urdu translation exists
   * Example: { "chapter-01/intro.md": true, "chapter-02/advanced.md": false }
   */
  [docPath: string]: boolean;
}

/**
 * Language configuration for Docusaurus localeConfigs
 */
export interface LanguageConfig {
  /**
   * Display label in language switcher
   */
  label: string;

  /**
   * Text direction
   */
  direction: TextDirection;

  /**
   * HTML lang attribute value (BCP 47 tag)
   */
  htmlLang: string;
}

/**
 * Map of language codes to their configurations
 */
export interface LanguageConfigMap {
  en: LanguageConfig;
  ur: LanguageConfig;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * localStorage key for language preference
 */
export const LANGUAGE_PREFERENCE_KEY = 'preferredLanguage' as const;

/**
 * URL query parameter name for language
 */
export const LANGUAGE_QUERY_PARAM = 'lang' as const;

/**
 * Default language (fallback)
 */
export const DEFAULT_LANGUAGE: LanguageCode = 'en';

/**
 * Language configuration constants
 */
export const LANGUAGE_CONFIGS: LanguageConfigMap = {
  en: {
    label: 'English',
    direction: 'ltr',
    htmlLang: 'en-US',
  },
  ur: {
    label: 'اردو',
    direction: 'rtl',
    htmlLang: 'ur-PK',
  },
};

// ============================================================================
// Error Types
// ============================================================================

/**
 * Custom error for language preference operations
 */
export class LanguagePreferenceError extends Error {
  constructor(
    message: string,
    public readonly code: 'STORAGE_UNAVAILABLE' | 'INVALID_LANGUAGE' | 'NAVIGATION_FAILED',
    public readonly originalError?: Error
  ) {
    super(message);
    this.name = 'LanguagePreferenceError';
  }
}
