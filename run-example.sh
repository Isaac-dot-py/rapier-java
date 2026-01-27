#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./run-example.sh <ExampleClassName>"
    echo "Available examples:"
    echo "  BouncingBallExample"
    echo "  MultiObjectExample"
    exit 1
fi

EXAMPLE_CLASS="com.rapier.example.$1"

java -cp "target/classes:example/target/classes:$(mvn dependency:build-classpath -q -DincludeScope=runtime -Dmdep.outputFile=/dev/stdout)" \
  "$EXAMPLE_CLASS"
