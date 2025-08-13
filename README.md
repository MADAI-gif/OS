# OS Lab Project  

## Introduction 🌐  
Explorez les fondements d’un **système d’exploitation multitâche** pour ARM grâce à ce projet conçu pour la carte **LPC55S69-EVK** de NXP. Tout le code s’exécute sur le **Core 0 Cortex-M33**, offrant une plateforme idéale pour expérimenter un OS personnalisé.  

### Organisation du Logiciel :  
1. **Code applicatif** *(Mode Thread unprivileged)* :  
   - Exécution de multiples tâches (*pseudo-parallèle*).  
   - Accès aux services OS via **appels systèmes (SVC)**.  
   - Gestion des interruptions activée.  

2. **Système d’exploitation** *(Mode Handler privileged & Thread unprivileged)* :  
   - **Noyau** :  
     - Commutation des tâches via le **timer système** (*round-robin*) ou des événements système.  
     - Gestion des sémaphores et des appels systèmes.  
   - **Drivers** :  
     - Communication standardisée avec les périphériques.  
     - Synchronisation via sémaphores pour un fonctionnement robuste.  

---

## Matériel Utilisé 🛠️  
- **Carte cible** : LPC55S69-EVK  
- **Périphériques intégrés** :  
  - **USART** : Interface série pour débogage.  
  - **Accéléromètre** : Capture des données en temps réel.  
  - **Carte SD** : Gestion de fichiers via FATFS.  

---

## Fonctionnalités 📋  
1. **Tâches Applicatives** :  
   - Création et gestion multitâche.  
   - Accès OS via appels systèmes (*SVC*).  

2. **Synchronisation & Communication** :  
   - Utilisation des **sémaphores** pour coordonner tâches et périphériques.  

3. **Planification et Commutation** :  
   - Mode **round-robin** et gestion basée sur événements.  

4. **Drivers Standardisés** :  
   - Communication avec les périphériques via une interface uniforme.  

5. **Système de Fichiers** :  
   - Gestion des fichiers avec FATFS sur carte SD.  

---

## Instructions pour l’Utilisation 🚀  
1. **Configuration Matérielle** :  
   - Connecter les périphériques (accéléromètre, carte SD) à la carte LPC55S69-EVK.  
   - Configurer la communication USART avec un PC pour le débogage.  

2. **Compilation et Tests** :  
   - Sélectionnez le test souhaité dans `main.c` en définissant un label (`#define MAIN_EXx`).  
   - Compiler et flasher le code sur la carte cible.  

3. **Validation des Résultats** :  
   - Utilisez un terminal série (e.g., PuTTY) pour surveiller les sorties et les logs.  
   - Vérifiez les interactions avec les périphériques connectés.  

---

## Labels Disponibles dans `main.c` 🎯  
- **`MAIN_TEST`** : Vérifie un appel système simple.  
- **`MAIN_EX1`** : Création de la première tâche et changement de tâche.  
- **`MAIN_EX2`** : Multi-tâches avec exécution d’un même code.  
- **`MAIN_EX3`** : Test des sémaphores.  
- **`MAIN_EX4`** : Utilisation des sémaphores comme mutex.  
- **`MAIN_EX5`** : Fonction de temporisation.  
- **`MAIN_EX6`** : Exécution de deux tâches temporisées.  
- **`MAIN_EX7`** : Test de terminaison d’une tâche.  
- **`MAIN_EX8`** : Interface de fichiers virtuels.  
- **`MAIN_EX9`** : LED RGB comme périphérique.  
- **`MAIN_EX10`** : Bouton utilisateur avec gestion d’interruptions.  

---

## Licence 📄  
Ce projet est sous licence MIT. Consultez le fichier `LICENSE` pour plus d'informations.
