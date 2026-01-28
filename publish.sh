#!/usr/bin/env bash
set -euo pipefail

PUBLISH_DIR="$(cd "$(dirname "$0")" && pwd)/build/m2"
mkdir -p "$PUBLISH_DIR"

echo "Publishing rapier-java to file://$PUBLISH_DIR"

GRADLE_BIN="./gradlew"
if [ ! -x "$GRADLE_BIN" ]; then
	GRADLE_BIN="gradle"
fi

$GRADLE_BIN -DskipTests publish

echo "Artifacts published to file://$PUBLISH_DIR"
