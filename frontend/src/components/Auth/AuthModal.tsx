/**
 * AuthModal - Handles Login, Signup, and Onboarding flows
 */

import React, { useState } from 'react';
import OnboardingWizard, { UserProfile } from './OnboardingWizard';
import styles from './styles.module.css';

interface AuthModalProps {
  onClose: () => void;
  initialView?: 'login' | 'signup';
}

const AuthModal: React.FC<AuthModalProps> = ({ onClose, initialView = 'login' }) => {
  const [view, setView] = useState<'login' | 'signup' | 'onboarding'>(initialView);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  const handleLogin = (e: React.FormEvent) => {
    e.preventDefault();
    console.log('Logging in:', { email, password });
    // TODO: Integrate better-auth client here
    onClose();
  };

  const handleSignup = (e: React.FormEvent) => {
    e.preventDefault();
    console.log('Signing up:', { name, email, password });
    // TODO: Integrate better-auth signup here
    // On success:
    setView('onboarding');
  };

  const handleOnboardingComplete = (profile: UserProfile) => {
    console.log('Onboarding complete:', profile);
    // TODO: Save profile to Neon DB via API
    onClose();
  };

  // Stop click propagation to prevent closing when clicking inside modal
  const handleContentClick = (e: React.MouseEvent) => {
    e.stopPropagation();
  };

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={handleContentClick}>
        <button className={styles.closeButton} onClick={onClose}>×</button>

        {view === 'login' && (
          <>
            <h2 className={styles.title}>Welcome Back</h2>
            <p className={styles.subtitle}>Log in to access your personalized lab.</p>
            
            <form className={styles.form} onSubmit={handleLogin}>
              <div className={styles.inputGroup}>
                <label className={styles.label}>Email</label>
                <input 
                  type="email" 
                  className={styles.input} 
                  placeholder="pilot@robotics.lab"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              <div className={styles.inputGroup}>
                <label className={styles.label}>Password</label>
                <input 
                  type="password" 
                  className={styles.input} 
                  placeholder="••••••••"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>
              <button type="submit" className={styles.primaryButton}>Log In</button>
            </form>

            <p className={styles.switchMode}>
              New pilot? 
              <button className={styles.link} onClick={() => setView('signup')}>Initialize System</button>
            </p>
          </>
        )}

        {view === 'signup' && (
          <>
            <h2 className={styles.title}>Initialize System</h2>
            <p className={styles.subtitle}>Create your pilot profile.</p>
            
            <form className={styles.form} onSubmit={handleSignup}>
              <div className={styles.inputGroup}>
                <label className={styles.label}>Name</label>
                <input 
                  type="text" 
                  className={styles.input} 
                  placeholder="Ace Pilot"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
                />
              </div>
              <div className={styles.inputGroup}>
                <label className={styles.label}>Email</label>
                <input 
                  type="email" 
                  className={styles.input} 
                  placeholder="pilot@robotics.lab"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              <div className={styles.inputGroup}>
                <label className={styles.label}>Password</label>
                <input 
                  type="password" 
                  className={styles.input} 
                  placeholder="Create a secure key"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>
              <button type="submit" className={styles.primaryButton}>Create Account</button>
            </form>

            <p className={styles.switchMode}>
              Already initialized? 
              <button className={styles.link} onClick={() => setView('login')}>Log In</button>
            </p>
          </>
        )}

        {view === 'onboarding' && (
          <OnboardingWizard onComplete={handleOnboardingComplete} />
        )}
      </div>
    </div>
  );
};

export default AuthModal;
