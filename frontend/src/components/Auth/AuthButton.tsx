import React from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css'; // Assuming styles are shared

const AuthButton: React.FC = () => {
  const { user, openLogin, openSignup } = useAuth();

  if (user) {
    // If logged in, show user name or profile link
    return (
      <button className={styles.navAuthButton}>
        Hello, {user.name.split(' ')[0]}!
      </button>
    );
  }

  return (
    <>
      <button
        className={`${styles.navAuthButton} ${styles.navLogin}`}
        onClick={openLogin}
      >
        Log In
      </button>
      <button
        className={`${styles.navAuthButton} ${styles.navSignup}`}
        onClick={openSignup}
      >
        Sign Up
      </button>
    </>
  );
};

export default AuthButton;
