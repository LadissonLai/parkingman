#!/usr/bin/env python3
"""
Convert parking dataset JSON files to JSONL format for LLM training.
This script reads all JSON files from T1, T2, T3 folders and combines them into a single JSONL file.
"""

import json
import os
from pathlib import Path
import argparse


def convert_json_to_jsonl(dataset_dir: str, output_file: str):
    """
    Convert all JSON files in T1, T2, T3 folders to a single JSONL file.

    Args:
        dataset_dir: Path to the dataset directory containing T1, T2, T3 folders
        output_file: Path to the output JSONL file
    """
    dataset_path = Path(dataset_dir)

    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")

    total_files = 0
    processed_files = 0

    # Count total files first
    for subfolder in ['T1', 'T2', 'T3']:
        subfolder_path = dataset_path / subfolder
        if subfolder_path.exists():
            json_files = list(subfolder_path.glob('*.json'))
            total_files += len(json_files)
            print(f"Found {len(json_files)} files in {subfolder}")

    print(f"Total files to process: {total_files}")

    # Process files and write to JSONL
    with open(output_file, 'w', encoding='utf-8') as f:
        for subfolder in ['T1', 'T2', 'T3']:
            subfolder_path = dataset_path / subfolder
            if not subfolder_path.exists():
                print(f"Warning: Subfolder {subfolder} does not exist, skipping...")
                continue

            json_files = sorted(subfolder_path.glob('*.json'))

            for json_file in json_files:
                try:
                    # Read entire file content as one JSON object
                    with open(json_file, 'r', encoding='utf-8') as json_f:
                        file_content = json_f.read()

                    # Try to parse as JSON
                    try:
                        data = json.loads(file_content)
                        # Write to JSONL (one JSON object per line)
                        json.dump(data, f, ensure_ascii=False)
                        f.write('\n')
                        processed_files += 1
                    except json.JSONDecodeError:
                        # If parsing as single JSON fails, try parsing line by line
                        # (some files contain multiple JSON objects, one per line)
                        with open(json_file, 'r', encoding='utf-8') as json_f:
                            line_count = 0
                            for line_num, line in enumerate(json_f, 1):
                                line = line.strip()
                                if not line:  # Skip empty lines
                                    continue

                                try:
                                    data = json.loads(line)
                                    # Write to JSONL (one JSON object per line)
                                    json.dump(data, f, ensure_ascii=False)
                                    f.write('\n')
                                    line_count += 1
                                except json.JSONDecodeError as e:
                                    print(f"Error parsing line {line_num} in {json_file}: {e}")
                                    continue

                    processed_files += 1

                    if processed_files % 10 == 0:
                        print(f"Processed {processed_files}/{total_files} files")

                except json.JSONDecodeError as e:
                    print(f"Error parsing {json_file}: {e}")
                except Exception as e:
                    print(f"Error processing {json_file}: {e}")

    print(f"Successfully processed {processed_files}/{total_files} files")
    print(f"Output saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Convert parking dataset to JSONL format')
    parser.add_argument('--dataset_dir', type=str,
                       default='dataset',
                       help='Path to dataset directory (default: dataset)')
    parser.add_argument('--output', type=str,
                       default='parking_dataset.jsonl',
                       help='Output JSONL file path (default: parking_dataset.jsonl)')

    args = parser.parse_args()

    # If running from dataset_collector directory
    if os.path.basename(os.getcwd()) == 'dataset_collector':
        dataset_dir = args.dataset_dir
    else:
        # If running from project root, adjust path
        dataset_dir = os.path.join('src', 'LLMParking', 'dataset_collector', args.dataset_dir)

    output_file = args.output

    print(f"Dataset directory: {dataset_dir}")
    print(f"Output file: {output_file}")

    convert_json_to_jsonl(dataset_dir, output_file)


if __name__ == '__main__':
    main()
