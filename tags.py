#!/usr/bin/env python3
"""
Script to scan directories for config.yaml files and display tags in an ASCII table.
"""

import os
import yaml
from pathlib import Path


def find_config_files(root_dir):
    """Find all config.yaml files in subdirectories."""
    config_files = []
    root_path = Path(root_dir)
    
    for item in root_path.iterdir():
        if item.is_dir():
            config_file = item / 'config.yaml'
            if config_file.exists():
                config_files.append((item.name, config_file))
    
    return sorted(config_files)


def read_tags_from_config(config_path):
    """Read tags from a config.yaml file."""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            return config.get('tags', [])
    except Exception as e:
        print(f"Error reading {config_path}: {e}")
        return []


def collect_all_tags(directory_tags):
    """Collect all unique tags from all directories."""
    all_tags = set()
    for _, tags in directory_tags:
        all_tags.update(tags)
    return sorted(all_tags)


def print_ascii_table(directory_tags, all_tags):
    """Print an ASCII table showing directories and their tags."""
    # Calculate column widths
    dir_col_width = max(len("Regression test"), max(len(dir_name) for dir_name, _ in directory_tags)) + 2
    tag_col_width = max(len(tag) + 2 for tag in all_tags) if all_tags else 10
    
    # Print header
    header = f"{'Regression test':<{dir_col_width}}"
    for tag in all_tags:
        header += f"| {tag:^{tag_col_width}} "
    print(header)
    
    # Print separator
    separator = "-" * dir_col_width
    for _ in all_tags:
        separator += "+-" + "-" * tag_col_width + "-"
    print(separator)
    
    # Print rows
    for dir_name, tags in directory_tags:
        row = f"{dir_name:<{dir_col_width}}"
        for tag in all_tags:
            marker = "X" if tag in tags else ""
            row += f"| {marker:^{tag_col_width}} "
        print(row)


def main():
    """Main function to scan directories and display tags table."""
    # Use current directory as root
    root_dir = os.getcwd()
    
    # Find all config files
    config_files = find_config_files(root_dir)
    
    if not config_files:
        print("No config.yaml files found in subdirectories.")
        return
    
    # Read tags from each config file
    directory_tags = []
    for dir_name, config_path in config_files:
        tags = read_tags_from_config(config_path)
        directory_tags.append((dir_name, tags))
    
    # Collect all unique tags
    all_tags = collect_all_tags(directory_tags)
    
    if not all_tags:
        print("No tags found in any config.yaml files.")
        return
    
    # Print the table
    print_ascii_table(directory_tags, all_tags)


if __name__ == "__main__":
    main()
