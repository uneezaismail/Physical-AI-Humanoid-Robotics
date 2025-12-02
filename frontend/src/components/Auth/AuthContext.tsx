/**
 * AuthContext - Global state for Authentication
 */

import React, { createContext, useContext, useState } from 'react';
import { UserProfile } from './OnboardingWizard';

interface User {
  name: string;
  email: string;
  profile?: UserProfile;
}

interface AuthContextType {
  user: User | null;
  isModalOpen: boolean;
  openLogin: () => void;
  openSignup: () => void;
  closeModal: () => void;
  initialView: 'login' | 'signup';
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null); // Placeholder for better-auth user
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [initialView, setInitialView] = useState<'login' | 'signup'>('login');

  const openLogin = () => {
    setInitialView('login');
    setIsModalOpen(true);
  };

  const openSignup = () => {
    setInitialView('signup');
    setIsModalOpen(true);
  };

  const closeModal = () => {
    setIsModalOpen(false);
  };

  return (
    <AuthContext.Provider value={{ user, isModalOpen, openLogin, openSignup, closeModal, initialView }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
