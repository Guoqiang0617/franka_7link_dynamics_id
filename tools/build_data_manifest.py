#!/usr/bin/env python3
import csv
import argparse
import hashlib
import re
from collections import defaultdict
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PACKAGE_ROOT / "data"
FILE_PATTERN = re.compile(r"trajectory_(\d+)(?:_(\d{8}_\d{6}))?")


def trajectory_identity(path: Path):
    match = FILE_PATTERN.fullmatch(path.stem)
    if not match:
        return None, None
    return int(match.group(1)), match.group(2)


def index_json_files(directory: Path):
    index = defaultdict(list)
    for path in directory.glob("trajectory_*.json"):
        trajectory_id, timestamp = trajectory_identity(path)
        if trajectory_id is not None:
            index[trajectory_id].append((timestamp, path))
    return index


def source_path_for(package_path: Path) -> str:
    relative = package_path.relative_to(DATA_ROOT)
    if relative.parts[:2] == ("raw", "train"):
        return f"ffs_data/train/{package_path.name}"
    if relative.parts[:2] == ("raw", "test"):
        return f"ffs_data/test/{package_path.name}"
    if relative.parts[:2] == ("raw", "unassigned"):
        return f"ffs_data/{package_path.name}"
    if relative.parts[:2] == ("trajectories", "train"):
        return f"ffs_data/train/{package_path.name}"
    if relative.parts[:2] == ("trajectories", "test"):
        return f"ffs_data/test/{package_path.name}"
    if relative.parts[:2] == ("trajectories", "summaries"):
        return f"ffs_data/{package_path.name}"
    return ""


def add_file_row(rows, package_path, file_type, split="", trajectory_id="", timestamp="", paired="", status=""):
    source_path = source_path_for(package_path)
    rows.append(
        {
            "file_type": file_type,
            "split": split,
            "trajectory_id": trajectory_id,
            "timestamp": timestamp or "",
            "package_path": package_path.relative_to(PACKAGE_ROOT).as_posix(),
            "source_path": source_path,
            "paired_package_path": paired,
            "status": status,
            "size_bytes": package_path.stat().st_size,
        }
    )


def main():
    parser = argparse.ArgumentParser(description="Rebuild the packaged snapshot inventory without accessing the original workspace.")
    parser.add_argument("--hashes", action="store_true", help="Read file contents and add SHA-256 hashes for copy verification.")
    args = parser.parse_args()
    rows = []
    train_json = index_json_files(DATA_ROOT / "trajectories" / "train")
    test_json = index_json_files(DATA_ROOT / "trajectories" / "test")

    for split in ("train", "test"):
        csv_dir = DATA_ROOT / "raw" / split
        json_index = train_json if split == "train" else test_json
        for csv_path in sorted(csv_dir.glob("trajectory_*.csv")):
            trajectory_id, timestamp = trajectory_identity(csv_path)
            candidates = json_index.get(trajectory_id, [])
            exact = [path for stamp, path in candidates if timestamp and stamp == timestamp]
            if exact:
                selected = exact[0]
                match_rule = "exact_timestamp"
            else:
                current = [
                    path for stamp, path in candidates
                    if stamp != "20260514_174449" or split == "train"
                ]
                selected = current[0] if len(current) == 1 else None
                match_rule = "trajectory_index_reuse" if selected else ""
            paired = (
                selected.relative_to(PACKAGE_ROOT).as_posix() if selected else ""
            )
            status = match_rule if selected else "no_matching_trajectory_json"
            add_file_row(
                rows,
                csv_path,
                "robot_state_csv",
                split,
                trajectory_id or "",
                timestamp or "",
                paired,
                status,
            )

    for csv_path in sorted((DATA_ROOT / "raw" / "unassigned").glob("*.csv")):
        trajectory_id, timestamp = trajectory_identity(csv_path)
        add_file_row(
            rows,
            csv_path,
            "robot_state_csv",
            "unassigned",
            trajectory_id or "",
            timestamp or "",
            "",
            "manual_review_required",
        )

    paired_json = {row["paired_package_path"] for row in rows if row["paired_package_path"]}
    for directory, split in (
        (DATA_ROOT / "trajectories" / "train", "train"),
        (DATA_ROOT / "trajectories" / "test", "test"),
        (DATA_ROOT / "trajectories" / "test" / "legacy_20260514_174449", "legacy_test"),
    ):
        for json_path in sorted(directory.glob("trajectory_*.json")):
            trajectory_id, timestamp = trajectory_identity(json_path)
            package_path = json_path.relative_to(PACKAGE_ROOT).as_posix()
            status = "paired_to_collected_csv" if package_path in paired_json else "generated_without_collected_csv"
            add_file_row(
                rows,
                json_path,
                "ffs_parameters_json",
                split,
                trajectory_id or "",
                timestamp or "",
                "",
                status,
            )

    for summary_path in sorted((DATA_ROOT / "trajectories" / "summaries").glob("*.json*")):
        add_file_row(rows, summary_path, "trajectory_summary", "", "", "", "", "source_snapshot")

    output = DATA_ROOT / "MANIFEST.csv"
    if args.hashes:
        for index, row in enumerate(rows):
            digest = hashlib.sha256()
            with (PACKAGE_ROOT / row["package_path"]).open("rb") as stream:
                for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                    digest.update(block)
            row["sha256"] = digest.hexdigest()
            if index % 100 == 0:
                print(f"Hashed {index + 1}/{len(rows)} files", flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        columns = [
            "file_type",
            "split",
            "trajectory_id",
            "timestamp",
            "package_path",
            "source_path",
            "paired_package_path",
            "status",
            "size_bytes",
        ]
        if args.hashes:
            columns.append("sha256")
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} records to {output}")


if __name__ == "__main__":
    main()
