#!/bin/bash

# Staging aller Änderungen
git add .

# Abfrage der Commit-Nachricht
echo "Geben Sie Ihre Commit-Nachricht ein: "
read commitMessage

# Commit mit der eingegebenen Nachricht
git commit -m "$commitMessage"

# Push zu origin
git push
