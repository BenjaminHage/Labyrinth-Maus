#!/bin/bash

# Staging aller Änderungen
git add .

# Eingabe der Commit-Nachricht mit dialog
commitMessage=$(dialog --inputbox "Geben Sie Ihre Commit-Nachricht ein:" 10 50 3>&1 1>&2 2>&3)

# Überprüfe, ob die Commit-Nachricht leer ist
if [ -z "$commitMessage" ]; then
    echo "Commit-Nachricht darf nicht leer sein."
    exit 1
fi

# Commit mit der eingegebenen Nachricht
git commit -m "$commitMessage"

# Push zu origin
git push
