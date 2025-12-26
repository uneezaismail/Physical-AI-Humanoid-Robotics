#!/usr/bin/env python3
"""
Content validation script for Physical AI & Humanoid Robotics textbook.

Validates MDX files for required structure:
- Intro section
- Learning Objectives section
- Hook section
- Concept section
- Mermaid Diagram
- Code Example
- Exercises
- Summary
- Preview Next Chapter
"""

import os
import sys
import re
from pathlib import Path
from typing import List, Dict, Tuple
import frontmatter  # type: ignore


def validate_chapter_structure(content: str) -> Tuple[bool, List[str]]:
    """
    Validates the chapter structure in the provided content.

    Args:
        content: The chapter content as a string

    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    errors = []
    warnings = []  # Track pedagogical quality issues as warnings

    # Check for required sections
    required_sections = [
        r'^## Intro',
        r'^## Learning Objectives',
        r'^## Hook',
        r'^## Concept',
        r'^## Mermaid Diagram',
        r'^## Code Example',
        r'^## Exercises',
        r'^## Summary',
        r'^## Preview Next Chapter'
    ]

    for section_pattern in required_sections:
        if not re.search(section_pattern, content, re.MULTILINE):
            section_name = section_pattern.replace(r'^## ', '').replace(r'$', '')
            errors.append(f"Missing required section: {section_name}")

    # Check for learning outcomes in frontmatter
    try:
        post = frontmatter.loads(content)
        learning_outcomes = post.get('learning_outcomes', [])
        if not learning_outcomes or len(learning_outcomes) < 3:
            errors.append("Learning outcomes should have at least 3 items based on Bloom's Taxonomy")
        else:
            # Check if learning outcomes follow Bloom's Taxonomy levels
            bloom_verbs = {
                'remember': ['define', 'describe', 'identify', 'label', 'list', 'name', 'recall', 'recognize'],
                'understand': ['explain', 'interpret', 'summarize', 'classify', 'describe', 'discuss', 'estimate'],
                'apply': ['apply', 'calculate', 'demonstrate', 'illustrate', 'solve', 'use', 'implement'],
                'analyze': ['analyze', 'compare', 'contrast', 'differentiate', 'organize', 'relate', 'categorize'],
                'evaluate': ['appraise', 'argue', 'assess', 'critique', 'evaluate', 'justify', 'support'],
                'create': ['design', 'assemble', 'construct', 'create', 'develop', 'formulate', 'plan']
            }

            bloom_found = set()
            for outcome in learning_outcomes:
                outcome_lower = outcome.lower()
                for level, verbs in bloom_verbs.items():
                    if any(verb in outcome_lower for verb in verbs):
                        bloom_found.add(level)

            if len(bloom_found) < 2:
                warnings.append(f"Learning outcomes should cover at least 2 different Bloom's Taxonomy levels, found: {list(bloom_found) if bloom_found else ['none']}")

        # Check prerequisites
        prerequisites = post.get('prerequisites', [])
        if not prerequisites:
            errors.append("Prerequisites should be specified in frontmatter")
        elif len(prerequisites) < 2:
            warnings.append("Consider adding more specific prerequisites to help students prepare")

    except Exception as e:
        errors.append(f"Error parsing frontmatter: {str(e)}")

    # Check for Mermaid diagram syntax
    if '```mermaid' not in content:
        errors.append("No Mermaid diagram found in chapter")

    # Check for code example syntax
    if '```python' not in content and '```' not in content.replace('```mermaid', ''):
        errors.append("No code example found in chapter")
    else:
        # Validate code examples
        python_code_blocks = re.findall(r'```python\s*\n(.*?)```', content, re.DOTALL)
        all_code_blocks = re.findall(r'```\s*\n(.*?)```', content, re.DOTALL)

        # Check if code blocks have proper explanations
        for i, code_block in enumerate(python_code_blocks):
            # Check if there's explanatory text before the code block
            code_start = content.find(f'```python{code_block}```')
            preceding_text = content[:code_start]
            lines_before = preceding_text.split('\n')[-5:]  # Look at last 5 lines before code

            has_explanation = any(line.strip() and not line.strip().startswith('#')
                                for line in lines_before if line.strip() and '```' not in line)

            if not has_explanation:
                warnings.append(f"Code example #{i+1} should have explanatory text before it")

        # Check code quality
        for i, code_block in enumerate(python_code_blocks):
            if len(code_block.strip()) < 20:
                warnings.append(f"Code example #{i+1} appears to be too short for a meaningful example")

            # Check for basic Python syntax issues (simple heuristics)
            # For educational content, we'll be less strict about indentation in examples
            # since code might be simplified for learning purposes
            lines = code_block.split('\n')
            for j, line in enumerate(lines):
                if line.strip().endswith(':') and j + 1 < len(lines):
                    next_line = lines[j + 1]
                    # For Python, after a colon the next line should typically be indented
                    # Skip if the next line is empty, already indented, or a comment/docstring
                    if next_line.strip() and not next_line.strip().startswith((' ', '\t')) and not next_line.strip().startswith(('#', '"""', "'''")):
                        # This could be a real indentation issue, but for educational examples
                        # we might want to be more lenient
                        stripped_next = next_line.strip()
                        # Only warn if it's clearly not a valid Python construct after colon
                        if not any(stripped_next.startswith(kw) for kw in ['def ', 'class ', 'if ', 'for ', 'while ', 'with ', 'try:', 'except', 'finally:', 'else:', 'elif ', 'import ', 'from ', 'return ', 'pass', 'continue', 'break', '@']):
                            warnings.append(f"Possible indentation issue in code example #{i+1} at line {j+2}")

    # Check for accessibility features in diagrams
    if '```mermaid' in content and 'figcaption' not in content.lower():
        warnings.append("Consider adding figcaption with description for Mermaid diagrams to improve accessibility")

    # Check for exercises with hints/solutions
    if '## Exercises' in content and 'hint' not in content.lower() and 'solution' not in content.lower():
        warnings.append("Consider adding hints and solutions to exercises for better learning support")

    # Check for summary section quality
    if '## Summary' in content:
        # Find the summary section content
        summary_match = re.search(r'## Summary\s*\n(.*?)(?=\n## \w+|$)', content, re.DOTALL)
        if summary_match:
            summary_content = summary_match.group(1).strip()
            if len(summary_content.split()) < 50:
                warnings.append("Summary section should be more comprehensive (currently too brief)")

    # Check for preview next chapter quality
    if '## Preview Next Chapter' in content:
        preview_match = re.search(r'## Preview Next Chapter\s*\n(.*?)(?=\n## \w+|$)', content, re.DOTALL)
        if preview_match:
            preview_content = preview_match.group(1).strip()
            if len(preview_content.split()) < 30:
                warnings.append("Preview Next Chapter section should provide more detail about upcoming content")

    # Check word count (approximately)
    text_only = re.sub(r'```.*?```', '', content, flags=re.DOTALL)  # Remove code blocks
    text_only = re.sub(r'<!--.*?-->', '', text_only, flags=re.DOTALL)  # Remove comments
    words = len(text_only.split())

    if words < 800:
        errors.append(f"Word count too low: {words} words (minimum 800)")
    elif words > 1500:
        errors.append(f"Word count too high: {words} words (maximum 1500)")

    # For educational content, only critical issues should cause validation failure
    # Indentation warnings in code examples are treated as warnings, not errors
    critical_warnings = []
    indentation_warnings = []

    for warning in warnings:
        if "Possible indentation issue" in warning:
            indentation_warnings.append(warning)
        else:
            critical_warnings.append(warning)

    # Add only critical warnings as errors, indentation warnings are just warnings for educational content
    errors.extend(critical_warnings)

    return len(errors) == 0, errors


def validate_file(file_path: Path) -> Tuple[bool, List[str]]:
    """
    Validates a single MDX file.

    Args:
        file_path: Path to the MDX file

    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        is_valid, errors = validate_chapter_structure(content)
        return is_valid, errors

    except Exception as e:
        return False, [f"Error reading file {file_path}: {str(e)}"]


def validate_directory(directory: Path) -> Dict[str, Tuple[bool, List[str]]]:
    """
    Validates all MDX files in a directory recursively.

    Args:
        directory: Directory to scan for MDX files

    Returns:
        Dictionary mapping file paths to validation results
    """
    results = {}

    for mdx_file in directory.rglob("*.mdx"):
        is_valid, errors = validate_file(mdx_file)
        results[str(mdx_file)] = (is_valid, errors)

    return results


def main():
    """Main function to run the validation."""
    if len(sys.argv) != 2:
        print("Usage: python validate_content.py <directory_or_file>")
        sys.exit(1)

    path_str = sys.argv[1]
    path_obj = Path(path_str)

    if not path_obj.exists():
        print(f"Error: Path {path_str} does not exist")
        sys.exit(1)

    if path_obj.is_file():
        is_valid, errors = validate_file(path_obj)
        if is_valid:
            print(f"[VALID] {path_str} is valid")
        else:
            print(f"[INVALID] {path_str} has {len(errors)} error(s):")
            for error in errors:
                print(f"  - {error}")
            sys.exit(1)

    elif path_obj.is_dir():
        results = validate_directory(path_obj)
        total_files = len(results)
        valid_files = sum(1 for is_valid, _ in results.values() if is_valid)
        invalid_files = total_files - valid_files

        print(f"Validated {total_files} file(s): {valid_files} valid, {invalid_files} invalid")

        for file_path, (is_valid, errors) in results.items():
            if is_valid:
                print(f"[VALID] {file_path}")
            else:
                print(f"[INVALID] {file_path}")
                for error in errors:
                    print(f"    - {error}")

        if invalid_files > 0:
            sys.exit(1)


if __name__ == "__main__":
    main()