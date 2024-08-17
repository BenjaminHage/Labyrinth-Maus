#!/bin/bash

# Staging aller Änderungen
git add .

# Eingabe der Commit-Nachricht
echo "Geben Sie Ihre Commit-Nachricht ein: "
read commitMessage

# Überprüfe, ob die Commit-Nachricht leer ist
if [ -z "$commitMessage" ]; then
    echo "Commit-Nachricht darf nicht leer sein."
    exit 1
fi

# Commit mit der eingegebenen Nachricht
git commit -m "$commitMessage"

# Push zu origin
git push
