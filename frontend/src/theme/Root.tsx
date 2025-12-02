/**
 * Root wrapper - Adds global components to all pages
 */

import React, { useEffect } from "react";
import FloatingChatButton from "../components/FloatingChatButton";
import { AuthProvider, useAuth } from "../components/Auth/AuthContext";
import AuthModal from "../components/Auth/AuthModal";

// This is the component that will be wrapped by AuthProvider
const RootContent = ({
  children,
}: {
  children: React.ReactNode;
}): JSX.Element => {
  const { isModalOpen, closeModal, initialView, openLogin, openSignup } =
    useAuth();

  // Expose auth functions globally for Docusaurus config HTML items
  useEffect(() => {
    window.openLoginModal = openLogin;
    window.openSignupModal = openSignup;
    return () => {
      delete window.openLoginModal;
      delete window.openSignupModal;
    };
  }, [openLogin, openSignup]);

  return (
    <>
      {children}
      <FloatingChatButton />
      {isModalOpen && (
        <AuthModal onClose={closeModal} initialView={initialView} />
      )}
    </>
  );
};

export default function Root({
  children,
}: {
  children: React.ReactNode;
}): JSX.Element {
  return (
    <AuthProvider>
      {" "}
      {/* Wrap with ChatProvider */}
      <RootContent>{children}</RootContent>
    </AuthProvider>
  );
}

// Extend Window interface for TypeScript
declare global {
  interface Window {
    openLoginModal: () => void;
    openSignupModal: () => void;
  }
}
