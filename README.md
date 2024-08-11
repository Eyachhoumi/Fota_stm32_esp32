     FOTA (Mise à jour de Firmware Over-The-Air) pour STM32 et ESP32

Ce projet démontre un système de mise à jour de firmware Over-The-Air (FOTA) pour les microcontrôleurs STM32 en utilisant un ESP32 pour la connectivité WiFi et MQTT pour la communication. L'objectif principal est de permettre des mises à jour de firmware sans fil et transparentes pour un microcontrôleur STM32F407, avec le module ESP32 prenant en charge le téléchargement du nouveau firmware.
Fonctionnalités

    Mises à jour de firmware sans fil : Mettre à jour automatiquement le firmware sur un microcontrôleur STM32F407 via un message MQTT reçu par le module ESP32.
    Connectivité WiFi avec ESP32 : Utilisation du module ESP32 pour se connecter à un réseau WiFi et s'abonner à des sujets MQTT.
    Validation CRC : Assurer l'intégrité du firmware en calculant et en vérifiant les valeurs CRC lors du processus de mise à jour du firmware.
    Téléchargement HTTP : Télécharger les binaires du firmware en utilisant des liens HTTP reçus via MQTT, avec l'ESP32 prenant en charge le téléchargement des fichiers.

Flux de Travail du Projet

    Téléchargement du Firmware : Le nouveau binaire du firmware est téléchargé sur un serveur MinIO.
    Notification MQTT : Jenkins publie un message MQTT contenant le lien de téléchargement HTTP et la valeur CRC.
    Abonnement ESP32 : Le module ESP32 s'abonne au sujet MQTT et reçoit le message contenant le lien HTTP et la valeur CRC.
    Téléchargement par l'ESP32 : L'ESP32 télécharge le binaire du firmware en utilisant le lien HTTP reçu.
    Transfert au STM32 : Une fois le téléchargement terminé, l'ESP32 transfère le fichier binaire au STM32 via une interface de communication (UART/I2C).
    Vérification et Exécution : Le STM32 vérifie l'intégrité du firmware en utilisant la valeur CRC reçue et exécute le nouveau firmware si la vérification est réussie.

Prérequis

    Carte STM32F407 Discovery
    Module ESP32
    Serveur MinIO
    Broker MQTT Mosquitto
    Serveur CI Jenkins
    STM32CubeIDE et Arduino IDE
