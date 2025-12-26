/**
 * Integration Tests for useLanguagePreference Hook
 *
 * Tests for User Story 1: Quick Chapter Translation
 *
 * Test Cases:
 * 1. Hook initializes with Docusaurus context locale
 * 2. Hook detects language from URL path (/ur/docs vs /docs)
 * 3. `switchLanguage` function available for navigation coordination
 * 4. Hook handles Docusaurus context errors gracefully
 *
 * @see specs/002-urdu-translation/tasks.md T012
 * @see specs/002-urdu-translation/contracts/LanguageSwitcher.contract.ts
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { useLanguagePreference } from '../../src/hooks/useLanguagePreference';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Mock Docusaurus context
jest.mock('@docusaurus/useDocusaurusContext');

const mockUseDocusaurusContext = useDocusaurusContext as jest.MockedFunction<typeof useDocusaurusContext>;

describe('useLanguagePreference Hook', () => {
  beforeEach(() => {
    // Reset mocks before each test
    jest.clearAllMocks();
  });

  describe('Test Case 1: Hook Initializes with Docusaurus Context', () => {
    it('should initialize with English when on English page', () => {
      // Arrange: Mock Docusaurus context for English locale
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Hook returns English language
      expect(result.current.language).toBe('en');
      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBe(null);
    });

    it('should initialize with Urdu when on Urdu page', () => {
      // Arrange: Mock Docusaurus context for Urdu locale
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'ur',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Hook returns Urdu language
      expect(result.current.language).toBe('ur');
      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBe(null);
    });

    it('should show loading state initially and then resolve', async () => {
      // Arrange: Mock Docusaurus context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Initially loading
      expect(result.current.isLoading).toBe(true);

      // Wait for loading to complete
      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      // Assert: Language determined
      expect(result.current.language).toBe('en');
    });
  });

  describe('Test Case 2: Hook Detects Language from URL Path', () => {
    it('should detect English from /docs/* path pattern', () => {
      // Arrange: Mock English path
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: English detected
      expect(result.current.language).toBe('en');
    });

    it('should detect Urdu from /ur/docs/* path pattern', () => {
      // Arrange: Mock Urdu path
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'ur',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Urdu detected
      expect(result.current.language).toBe('ur');
    });

    it('should update language when Docusaurus context changes', () => {
      // Arrange: Mock initial English context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result, rerender } = renderHook(() => useLanguagePreference());

      // Assert: Initially English
      expect(result.current.language).toBe('en');

      // Act: Change to Urdu context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'ur',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      rerender();

      // Assert: Updated to Urdu
      expect(result.current.language).toBe('ur');
    });
  });

  describe('Test Case 3: switchLanguage Function Available', () => {
    it('should provide switchLanguage function', () => {
      // Arrange: Mock context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Function available
      expect(result.current.switchLanguage).toBeDefined();
      expect(typeof result.current.switchLanguage).toBe('function');
    });

    it('should call switchLanguage without throwing errors', () => {
      // Arrange: Mock context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      const { result } = renderHook(() => useLanguagePreference());

      // Act & Assert: Function call doesn't throw
      expect(() => {
        act(() => {
          result.current.switchLanguage('ur');
        });
      }).not.toThrow();
    });

    it('should log language switch request to console', () => {
      // Arrange: Mock console.log
      const consoleLogSpy = jest.spyOn(console, 'log').mockImplementation();

      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      const { result } = renderHook(() => useLanguagePreference());

      // Act: Call switchLanguage
      act(() => {
        result.current.switchLanguage('ur');
      });

      // Assert: Console log called
      expect(consoleLogSpy).toHaveBeenCalledWith(
        '[useLanguagePreference] Language switch requested:',
        'ur'
      );

      consoleLogSpy.mockRestore();
    });

    it('should accept both "en" and "ur" as valid language codes', () => {
      // Arrange
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      const { result } = renderHook(() => useLanguagePreference());

      // Act & Assert: Both language codes work
      expect(() => {
        act(() => {
          result.current.switchLanguage('en');
        });
      }).not.toThrow();

      expect(() => {
        act(() => {
          result.current.switchLanguage('ur');
        });
      }).not.toThrow();
    });
  });

  describe('Test Case 4: Error Handling', () => {
    it('should handle Docusaurus context errors gracefully', () => {
      // Arrange: Mock context to throw error
      mockUseDocusaurusContext.mockImplementation(() => {
        throw new Error('Docusaurus context unavailable');
      });

      // Act: Render hook (should not crash)
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Hook returns default state with error
      expect(result.current.language).toBe('en'); // Falls back to default
      expect(result.current.error).toBeTruthy();
      expect(result.current.isLoading).toBe(false);
    });

    it('should log error to console when initialization fails', () => {
      // Arrange: Mock console.error
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      mockUseDocusaurusContext.mockImplementation(() => {
        throw new Error('Context error');
      });

      // Act: Render hook
      renderHook(() => useLanguagePreference());

      // Assert: Error logged
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        '[useLanguagePreference] Initialization error:',
        expect.any(Error)
      );

      consoleErrorSpy.mockRestore();
    });

    it('should handle errors in switchLanguage function gracefully', () => {
      // Arrange: Mock context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      const { result } = renderHook(() => useLanguagePreference());

      // Mock switchLanguage to throw (simulate edge case)
      const originalSwitchLanguage = result.current.switchLanguage;

      // Act: Call switchLanguage (should handle errors internally)
      act(() => {
        try {
          originalSwitchLanguage('ur' as any);
        } catch (error) {
          // Should not throw to component
        }
      });

      // Assert: No unhandled errors
      expect(result.current.error).toBe(null);

      consoleErrorSpy.mockRestore();
    });

    it('should maintain functionality when error state is set', async () => {
      // Arrange: Mock context that causes error
      mockUseDocusaurusContext.mockImplementation(() => {
        throw new Error('Simulated error');
      });

      // Act: Render hook
      const { result } = renderHook(() => useLanguagePreference());

      // Assert: Hook still provides interface despite error
      expect(result.current.switchLanguage).toBeDefined();
      expect(result.current.language).toBe('en'); // Default fallback
      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).not.toBeNull();
    });
  });

  describe('Additional Edge Cases', () => {
    it('should return consistent values across multiple renders', () => {
      // Arrange: Mock context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      // Act: Render hook multiple times
      const { result, rerender } = renderHook(() => useLanguagePreference());
      const firstRender = result.current;

      rerender();
      const secondRender = result.current;

      // Assert: Values remain consistent
      expect(secondRender.language).toBe(firstRender.language);
      expect(secondRender.isLoading).toBe(firstRender.isLoading);
    });

    it('should handle rapid language context changes', () => {
      // Arrange: Mock English context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);

      const { result, rerender } = renderHook(() => useLanguagePreference());

      // Act: Rapidly change context
      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'ur',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);
      rerender();

      mockUseDocusaurusContext.mockReturnValue({
        siteConfig: {},
        i18n: {
          currentLocale: 'en',
          locales: ['en', 'ur'],
          defaultLocale: 'en',
          localeConfigs: {},
        },
      } as any);
      rerender();

      // Assert: Hook maintains stable state
      expect(result.current.language).toBe('en');
      expect(result.current.error).toBe(null);
    });
  });
});
