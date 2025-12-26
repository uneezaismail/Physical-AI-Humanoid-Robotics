/**
 * TypeScript API Contract: LanguageSwitcher Component
 *
 * Feature: 002-urdu-translation (Interactive Urdu Translation Button)
 * Date: 2025-12-18
 * Phase: 1 (Design & Contracts)
 *
 * This file defines the TypeScript interfaces and type contracts for the
 * LanguageSwitcher component and related utilities. All implementations
 * MUST conform to these contracts.
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
// Utility Function Contracts
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

  /**
   * Icon or flag to display (optional)
   */
  icon?: React.ReactNode;
}

/**
 * Map of language codes to their configurations
 */
export interface LanguageConfigMap {
  en: LanguageConfig;
  ur: LanguageConfig;
}

// ============================================================================
// Function Signatures
// ============================================================================

/**
 * Get initial language based on precedence: URL param > localStorage > default
 * @returns Initial language code
 */
export type GetInitialLanguage = () => LanguageCode;

/**
 * Type guard to validate language code at runtime
 * @param value - Value to check
 * @returns True if value is a valid LanguageCode
 */
export type IsValidLanguageCode = (value: unknown) => value is LanguageCode;

/**
 * Parse language code with fallback to default
 * @param value - Value to parse
 * @param fallback - Fallback language if parsing fails (default: 'en')
 * @returns Parsed language code or fallback
 */
export type ParseLanguageCode = (value: unknown, fallback?: LanguageCode) => LanguageCode;

/**
 * Get text direction for a given language
 * @param language - Language code
 * @returns Text direction ('ltr' or 'rtl')
 */
export type GetTextDirection = (language: LanguageCode) => TextDirection;

/**
 * Check if translation is available for a specific doc path
 * @param manifest - Translation manifest
 * @param docPath - Relative path to document
 * @returns True if Urdu translation exists
 */
export type IsTranslationAvailable = (
  manifest: TranslationManifest,
  docPath: string
) => boolean;

/**
 * Safe localStorage getter with error handling
 * @param key - localStorage key
 * @returns Value from localStorage or null if unavailable/error
 */
export type SafeLocalStorageGet = (key: string) => string | null;

/**
 * Safe localStorage setter with error handling
 * @param key - localStorage key
 * @param value - Value to store
 * @returns True if successful, false if error
 */
export type SafeLocalStorageSet = (key: string, value: string) => boolean;

/**
 * Update URL query parameter without page reload
 * @param key - Parameter key ('lang')
 * @param value - Language code value
 */
export type UpdateURLParameter = (key: string, value: string) => void;

/**
 * Navigate to equivalent page in different locale
 * Uses Docusaurus router to switch between /docs/page and /ur/docs/page
 * @param newLanguage - Target language code
 * @param currentPath - Current page path
 */
export type NavigateToLocale = (newLanguage: LanguageCode, currentPath: string) => void;

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
// Component Ref Types
// ============================================================================

/**
 * Ref type for LanguageSwitcher component
 * Exposes imperative handle for programmatic language switching
 */
export interface LanguageSwitcherRef {
  /**
   * Programmatically switch language
   */
  switchLanguage: (newLanguage: LanguageCode) => void;

  /**
   * Get current language
   */
  getCurrentLanguage: () => LanguageCode;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Custom event detail for language change events
 */
export interface LanguageChangeEventDetail {
  /**
   * Previous language code
   */
  previousLanguage: LanguageCode;

  /**
   * New language code
   */
  newLanguage: LanguageCode;

  /**
   * Source of the language change
   */
  source: 'button_click' | 'url_parameter' | 'localStorage' | 'default';

  /**
   * Timestamp of the change (ISO 8601 string)
   */
  timestamp: string;
}

/**
 * Custom event type for language change
 */
export type LanguageChangeEvent = CustomEvent<LanguageChangeEventDetail>;

// ============================================================================
// Validation & Testing Types
// ============================================================================

/**
 * Test utility type for mocking translation manifest
 */
export interface MockTranslationManifest extends TranslationManifest {
  /**
   * Helper to set translation availability for testing
   */
  setAvailable?: (docPath: string, available: boolean) => void;
}

/**
 * Test utility type for mocking localStorage
 */
export interface MockLocalStorage {
  getItem: jest.Mock<string | null, [string]>;
  setItem: jest.Mock<void, [string, string]>;
  removeItem: jest.Mock<void, [string]>;
  clear: jest.Mock<void, []>;
}

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

// ============================================================================
// Exports
// ============================================================================

/**
 * Re-export all types for convenience
 */
export type {
  LanguageCode,
  TextDirection,
  LanguageSwitcherProps,
  LanguagePreferenceState,
  TranslationButtonState,
  UseLanguagePreferenceReturn,
  TranslationManifest,
  LanguageConfig,
  LanguageConfigMap,
  GetInitialLanguage,
  IsValidLanguageCode,
  ParseLanguageCode,
  GetTextDirection,
  IsTranslationAvailable,
  SafeLocalStorageGet,
  SafeLocalStorageSet,
  UpdateURLParameter,
  NavigateToLocale,
  LanguageSwitcherRef,
  LanguageChangeEventDetail,
  LanguageChangeEvent,
  MockTranslationManifest,
  MockLocalStorage,
};

export { LanguagePreferenceError };
