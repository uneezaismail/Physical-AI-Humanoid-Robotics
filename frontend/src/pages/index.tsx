import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

import BookHighlights from "../components/BookHighlights";

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Physical AI & Humanoid Robotics Textbook"
    >
      <main className={styles.heroSection}>
        {/* Background Elements */}
        <div className={styles.bgHexagons}></div>
        <div className={styles.bgGlow}></div>

        <div className={styles.container}>
          {/* Text Content */}
          <div className={styles.content}>
            <Heading as="h1" className={styles.bookTitle}>
              Physical AI &<br />
              <span className={styles.titleGradient}>Humanoid Robotics</span>
            </Heading>
            <p className={styles.subtitle}>
              A next-generation textbook for learning <br />
              embodied intelligence, robotics, and AI systems.
            </p>
            <Link
              className={styles.startReadingBtn}
              to="/docs/part-1-foundations-lab/chapter-1-embodied-ai"
            >
              Start Reading
            </Link>
          </div>

          {/* Image Content */}
          <div className={styles.imageContainer}>
            {/* Holographic Circle Effect behind head */}
            <div className={styles.holoCircle}></div>

            {/* Robotic Screens & Chips */}
            <div className={`${styles.holoScreen} ${styles.screenLeft}`}>
              <div className={styles.screenHeader}>SYSTEM_DIAGNOSTICS</div>
              <div className={styles.dataLines}>
                <span>&gt; CPU_CORE: NORMAL</span>
                <span>&gt; MOTOR_DRIVER: OK</span>
                <span style={{ color: "#4de1c1" }}>&gt; VISION: ACTIVE</span>
              </div>
            </div>

            <div className={`${styles.holoScreen} ${styles.screenRight}`}>
              <div className={styles.screenHeader}>TARGET_LOCK</div>
              <div className={styles.targetReticle}></div>
            </div>

            <div className={`${styles.techChip} ${styles.chip1}`}></div>
            <div className={`${styles.techChip} ${styles.chip2}`}></div>

            <img
              src="img/home.png"
              alt="Physical AI Robot"
              className={styles.homeImage}
            />

            {/* Floating UI Elements (Decorative) */}
            <div
              className={styles.floatingUI}
              style={{ top: "20%", right: "-10%" }}
            ></div>
            <div
              className={styles.floatingUI}
              style={{ bottom: "30%", left: "-5%" }}
            ></div>
          </div>
        </div>
      </main>
      {/*<BookHighlights />*/}
    </Layout>
  );
}
