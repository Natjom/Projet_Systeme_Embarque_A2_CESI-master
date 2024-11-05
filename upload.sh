#!/bin/bash

# Définir les variables
FICHIER="src/main.ino"  # Chemin du fichier
CARTE="arduino:avr:uno"  # Modèle de votre carte
PORT="/dev/ttyUSB0"      # Remplacez par le port correspondant à votre carte si nécessaire

# Compilation
echo "Compilation de $FICHIER..."
arduino-cli compile --fqbn $CARTE $FICHIER

# Vérifier si la compilation a réussi
if [ $? -ne 0 ]; then
  echo "Erreur de compilation"
  exit 1
fi

# Téléversement
echo "Téléversement de $FICHIER sur le port $PORT..."
arduino-cli upload -p $PORT --fqbn $CARTE $FICHIER

# Vérifier si le téléversement a réussi
if [ $? -ne 0 ]; then
  echo "Erreur de téléversement"
  exit 1
fi

echo "Téléversement réussi !"
