/**
 * Better Auth configuration for Physical AI & Humanoid Robotics platform.
 *
 * This module configures authentication with:
 * - Email/password authentication
 * - Neon PostgreSQL database storage
 * - Separate user_profile table for questionnaire data
 */

import { betterAuth } from "better-auth";
import { Pool } from "pg";
import { config } from "./config.js";

const pool = new Pool({
  connectionString: config.databaseUrl,
  ssl: {
    rejectUnauthorized: false,
  },
});

export const auth = betterAuth({
  // Database configuration
  database: pool,

  // Trusted Origins
  trustedOrigins: [config.frontendUrl, "http://localhost:3000"],

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true,
  },

  // Session configuration (Remove sameSite from here, it doesn't belong here)
  session: {
    expiresIn: 60 * 60 * 24 * 7,
    updateAge: 60 * 60 * 24,
  },

  secret: config.betterAuthSecret!,
  baseURL: config.betterAuthUrl,

  // Advanced configuration (THIS IS THE FIX)
  advanced: {
    cookiePrefix: "physical-ai",
    // Force secure cookies to true (required for SameSite: None)
    useSecureCookies: true,
    // Explicitly define cookie attributes to allow cross-site usage
    defaultCookieAttributes: {
      sameSite: "none",
      secure: true,
      httpOnly: true,
    },
  },
});
