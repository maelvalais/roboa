/* MODULE PRINCIPAL DU LOGICIEL ROBOA (ROBOT OA) ASSURANT LE CONTROLE ET L'ASSERVISSEMENT DES MOUVEMENTS
 * DU NOUVEL INSTRUMENT ET LES DIFFERENTES ENTREES/SORTIES
 *
 * (d)09/2005 David.Romeuf@univ-lyon1.fr
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <openssl/bio.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "Flexmotn.h"
#include "ListesCommandesReponsesRobOA.h"


#define TAILLE_MAXI_CHAINE_ERREUR	1024
#define TAILLE_MAXI_CHAINE_ERREUR_SSL	1024

#define TAILLE_MAXI_CHAINE_BIO		1024

#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#define ListeClientsMax		10	/* Nombre d'entrees maxi de la liste des requetes de connexions entrantes      */
#define TimeoutSSLAccept	5	/* Timeout en seconde de l'initiation de la negociation TLS/SSL                */
#define TIMEOUT_SOCKET_PUT	20	/* Timeout en seconde d'une socket pour la fonction d'emission                 */
#define TIMEOUT_SOCKET_GET	240	/* Timeout en seconde d'une socket pour la fonction de reception               */

#define MessageBienvenue	"RobOA v1.0 : Bienvenue"

#define IPOkClient		0xC0A80601	/* Adresse IP permise pour etre client de ROBOA (format hote)          */

#define IdCarte	1		/* Id de la carte PCI7334 defini avec l'utilitaire "Measurement & Automation Explorer" */

#define NbPasCodeurAxe1	6000	/* Nombre de pas du codeur de l'axe 1                                                  */
#define NbPasCodeurAxe2	6000	/* Nombre de pas du codeur de l'axe 2                                                  */
#define NbPasCodeurAxe3	6000	/* Nombre de pas du codeur de l'axe 3                                                  */
#define NbPasCodeurAxe4	6000	/* Nombre de pas du codeur de l'axe 4                                                  */

#define NbPasMoteurAxe1	200	/* Nombre de pas du moteur de l'axe 1                                                  */
#define NbPasMoteurAxe2	200	/* Nombre de pas du moteur de l'axe 2                                                  */
#define NbPasMoteurAxe3	200	/* Nombre de pas du moteur de l'axe 3                                                  */
#define NbPasMoteurAxe4	200	/* Nombre de pas du moteur de l'axe 4                                                  */

#define NbMPAxe1	256	/* Nombre de micro pas configures sur le MID de l'axe 1                                */
#define NbMPAxe2	2	/* Nombre de micro pas configures sur le MID de l'axe 2                                */
#define NbMPAxe3	2	/* Nombre de micro pas configures sur le MID de l'axe 3                                */
#define NbMPAxe4	2	/* Nombre de micro pas configures sur le MID de l'axe 4                                */

#define VitesseMaxiAxe1	1000.0	/* Vitesse maximale de l'axe 1 en tours/minute                                         */
#define VitesseMaxiAxe2	60.0	/* Vitesse maximale de l'axe 2 en tours/minute                                         */
#define VitesseMaxiAxe3	1000.0	/* Vitesse maximale de l'axe 3 en tours/minute                                         */
#define VitesseMaxiAxe4	60.0	/* Vitesse maximale de l'axe 4 en tours/minute                                         */

#define AccelDecelMaxiAxe1	0.1	/* Acceleration maximale de l'axe 1 en tours/s par seconde                             */
#define AccelDecelMaxiAxe2	0.1	/* Acceleration maximale de l'axe 2 en tours/s par seconde                             */
#define AccelDecelMaxiAxe3	0.1	/* Acceleration maximale de l'axe 3 en tours/s par seconde                             */
#define AccelDecelMaxiAxe4	0.1	/* Acceleration maximale de l'axe 4 en tours/s par seconde                             */

#define TIMEOUT_RECH_INDEX_CODEUR	120000	/* Deux minutes pour trouver la position index des codeurs                     */
#define TIMEOUT_RECH_POSITION_REPOS	120000	/* Deux minutes pour trouver la position de repos                              */


/* Variables globales
*/
static char *MotDePasseClePriveeServeurROBOA="robot";
static char *IdentifieurContexteServeurROBOA="CTXServeurROBOA";
static char *ListeDesChiffreurs="HIGH";


/* Prototype des fonctions du module
*/
extern void CompChaineErreurFlexMotionNonModale(i32 CodeErreur,char *ChaineErreur);
extern void AffichagePurgeErreursModales(BIO *comm);
extern void AfficheErreurPileLibrairieSSL(void);
extern void AfficheErreurES_SSL(SSL *structure,int retour);
extern void FnHandlerSIGPIPE(int signal);
extern int EnvoyerChaineBIO(BIO *comm,char *chaine);
extern int InitialiseConfigureCarteMouvement(BIO *comm);
extern int ArretDouxDesactivationAxes(BIO *comm);
extern int RechercherIndexCodeur(u8 axe,u32 timeout,BIO *comm);
extern int RechercherPositionRepos(u8 axe,u32 timeout,BIO *comm);
extern int PositionsAxesEnPasCodeurQuadrature(i32 *PositionCodeur1,i32 *PositionCodeur2,i32 *PositionCodeur3,i32 *PositionCodeur4,BIO *comm);
extern int PositionsAxesEnPasMoteur(i32 *PositionCodeur1,i32 *PositionCodeur2,i32 *PositionCodeur3,i32 *PositionCodeur4,BIO *comm);
extern int PositionnerAxe(u8 axe,i32 destination);
extern int TestRecupCompChaineErreurFlexMotionModale(u16 *IdCommande,u16 *IdRessource,i32 *CodeErreurModale,char *ChaineErreur);
extern int FnMotDePasseCleChiffree(char *buf, int size, int rwflag, void *data);


int main(int argc,char *argv[])
{
	int ARRET_Machine=FALSE;	/* Drapeau pour indiquer si on doit stopper la machine                                   */
	int Sortir=FALSE;		/* Drapeau pour indiquer si on doit sortir de la boucle d'ecoute du serveur              */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale          */
	u16 IdCommandeErr;					/* Numero de commande d'une erreur modale                        */
	u16 IdRessourceErr;					/* Numero de ressource d'une erreur modale                       */
	i32 CodeErreurModaleErr;				/* Code d'erreur d'une erreur modale                             */
	char ChaineErreurModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur modale                        */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                    */
	const SSL_METHOD *MethodeVersionSSL;				/* Pointeur sur la methode SSL v2 ou V3                          */
	SSL_CTX *ContexteSSLROBOA;				/* Pointeur sur le contexte SSL, objet avec cle et certificats   */
	BIO *ObjetBIO;						/* Pointeur sur un objet BIO de la librairie OpenSSL             */
	DH *ParametresDH;					/* Pointeur sur les parametres aleatoires Diffie-Hellman         */
	int CodeVerifParametresDH;				/* Code retour de la verification des parametres DH              */
	int IdSocketServeurROBOA;				/* Identifieur du point de communication du serveur ROBOA        */
	int ParametreSocket;					/* Variable utilisee pour le parametrage d'une socket            */
	struct sockaddr_in AdresseSocketServeur;		/* Structure decrivant l'adresse:port d'attachement de la socket */
								/*  pour le domaine AF_INET                                      */

	
	/* Initialisation du contexte SSL qui contient les cles, les certificats a utiliser pour des connexions
	*/
	SSL_load_error_strings();	/* Chargement des chaines d'erreurs de la librairie                                                 */
	SSL_library_init();		/* Initialisation de la librairie OpenSSL : Chargement des differents algorithmes de chiffrement... */

	MethodeVersionSSL=SSLv23_method();	/* Methode du protocole SSL a utiliser : il existe aussi SSLv23_server_method()             */

	/* Creation d'un objet de contexte SSL selon une methode de version du protocole
	*/
	if( (ContexteSSLROBOA=SSL_CTX_new(MethodeVersionSSL)) == NULL )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_new() : impossible de creer le contexte SSL.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* Enregistrement dans le contexte SSL pour ROBOA de la fonction appelee lors d'un acces a un fichier PEM chiffre par un mot de passe
	 *  (typiquement pour l'acces a la cle privee, qui est conservee chiffre par la passphrase PEM, ou mot de passe)
	*/
	SSL_CTX_set_default_passwd_cb(ContexteSSLROBOA,FnMotDePasseCleChiffree);

	/* Chargement du certificat de l'autorite de certification de confiance (le CA) pour les OA dans le contexte SSL pour ROBOA
	*/
	if( !SSL_CTX_load_verify_locations(ContexteSSLROBOA,"/RobOA/ssl/CertificatCA_OA.pem","/etc/ssl/certs") )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_load_verify_locations(): impossible de charger le certificat de l'autorite de confiance pour les OA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* Chargement du certificat et donc de la cle publique du serveur ROBOA dans le contexte SSL pour ROBOA
	*/
	if( SSL_CTX_use_certificate_file(ContexteSSLROBOA,"/RobOA/ssl/CertificatServeurRobOA.pem",SSL_FILETYPE_PEM) != 1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_use_certificate_file(): impossible de charger le certificat (cle publique) du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* Chargement de la cle privee du serveur ROBOA dans le contexte SSL pour ROBOA
	*/
	if( SSL_CTX_use_PrivateKey_file(ContexteSSLROBOA,"/RobOA/ssl/ClePriveeServeurRobOA.pem",SSL_FILETYPE_PEM) != 1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_use_PrivateKey_file(): impossible de charger la cle privee du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* On verifie la conformite de la cle privee chargee
	*/
	if( !SSL_CTX_check_private_key(ContexteSSLROBOA) )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_check_private_key(): impossible de verifier la conformite de la cle privee du serveur ROBOA chargee dans le contexte.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* Chargement des parametres aleatoires Diffie-Hellman (dhparams) obtenus via la commande openssl. On utilise cette methode pour ameliorer
	 *  la vitesse de reponse du serveur avec un tres bon alea car la generation consomme beaucoup de temps.
	*/
	if( (ObjetBIO=BIO_new_file("/RobOA/ssl/Parametres-Diffie-Hellman-ServeurRobOA.pem","r")) == NULL )
	{
		fprintf(stderr,"ROBOA: ERREUR: BIO_new_file(): Impossible d'ouvrir le fichier des parametres Diffie-Hellman du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}
	
	if( (ParametresDH=PEM_read_bio_DHparams(ObjetBIO,NULL,NULL,NULL)) == NULL )
	{
		fprintf(stderr,"ROBOA: ERREUR: PEM_read_bio_DHparams(): Impossible de lire le fichier des parametres Diffie-Hellman du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}
	
	if( !BIO_free(ObjetBIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer ObjetBIO.\n");
	
	if( !DH_check(ParametresDH,&CodeVerifParametresDH) )
	{
		fprintf(stderr,"ROBOA: ERREUR: DH_check(): Erreur lors de la verification des parametres Diffie-Hellman du serveur ROBOA.\n");
		if( CodeVerifParametresDH & DH_CHECK_P_NOT_SAFE_PRIME ) fprintf(stderr,"ROBOA: ERREUR: DH_check(): DH_CHECK_P_NOT_SAFE_PRIME.\n");
		if( CodeVerifParametresDH & DH_NOT_SUITABLE_GENERATOR ) fprintf(stderr,"ROBOA: ERREUR: DH_check(): DH_NOT_SUITABLE_GENERATOR.\n");
		if( CodeVerifParametresDH & DH_UNABLE_TO_CHECK_GENERATOR ) fprintf(stderr,"ROBOA: ERREUR: DH_check(): DH_UNABLE_TO_CHECK_GENERATOR.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}
	
	if( !SSL_CTX_set_tmp_dh(ContexteSSLROBOA,ParametresDH) )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_set_tmp_dh(): Impossible de charger les parametres Diffie-Hellman dans le contexte du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* On parametre la liste des chiffreurs que le contexte peut utiliser (openssl ciphers -v 'HIGH')
	*/
	if( !SSL_CTX_set_cipher_list(ContexteSSLROBOA,ListeDesChiffreurs) )
	{
		fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_set_cipher_list(): Impossible de parametrer la liste des chiffreurs utilisables dans le contexte du serveur ROBOA.\n");
		AfficheErreurPileLibrairieSSL();
		exit(EXIT_FAILURE);
	}

	/* On parametre le type de verification imposee lors de la negociation, l'autentification des paires
	 *  SSL_VERIFY_PEER = Serveur : demande le certificat du client, Client : verification du certificat du serveur
	 *  SSL_VERIFY_FAIL_IF_NO_PEER_CERT = Serveur : si le client ne retourne pas son certificat la negociation immediatement arretee.
	*/
	SSL_CTX_set_verify(ContexteSSLROBOA,SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,NULL);

	/* Le contexte est cree et parametre, il faut maintenant lui donner un identifieur caracteristique pour pouvoir differencier les sessions
	 *  (dans le cas d'importation d'une session generee depuis un autre contexte par exemple)
	*/
	if( strlen(IdentifieurContexteServeurROBOA) < SSL_MAX_SSL_SESSION_ID_LENGTH )
	{
		if( !SSL_CTX_set_session_id_context(ContexteSSLROBOA,(const unsigned char*)IdentifieurContexteServeurROBOA,strlen(IdentifieurContexteServeurROBOA)) )
		{
			fprintf(stderr,"ROBOA: ERREUR: SSL_CTX_set_session_id_context(): Impossible de parametrer l'identifieur des sessions crees via le contexte du serveur ROBOA.\n");
			AfficheErreurPileLibrairieSSL();
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		fprintf(stderr,"ROBOA: ERREUR: Chaine identifieur sessions du contexte du serveur ROBOA trop longue.\n");
		exit(EXIT_FAILURE);
	}

	/* Creation d'un point de communication pour le serveur ROBOA, une socket du domaine IPv4 (PF_INET Linux), de type flux de donnees binaire
	 *  echange continu avec integrite et fiabilite maximale (SOCK_STREAM) et de protocole specifique IP (/etc/protocols)
	*/
	if( (IdSocketServeurROBOA=socket(PF_INET,SOCK_STREAM,0)) == -1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: socket(): Impossible d'obtenir une socket pour le serveur ROBOA, numero d'erreur %d.\n",errno);
		if( errno == EPROTONOSUPPORT ) fprintf(stderr,"ROBOA: ERREUR: socket(): Type de protocole ou protocole non disponible dans ce domaine de communication.\n");
		if( errno == EAFNOSUPPORT ) fprintf(stderr,"ROBOA: ERREUR: socket(): Famille d'adresses non supportee.\n");
		if( errno == ENFILE ) fprintf(stderr,"ROBOA: ERREUR: socket(): Table de descripteur par processus pleine.\n");
		if( errno == EMFILE ) fprintf(stderr,"ROBOA: ERREUR: socket(): Table des fichiers pleine.\n");
		if( errno == EACCES ) fprintf(stderr,"ROBOA: ERREUR: socket(): Creation d'une telle socket non autorise.\n");
		if( errno == ENOBUFS || errno == ENOMEM ) fprintf(stderr,"ROBOA: ERREUR: socket(): Pas assez de memoire pour allouer les buffers necessaires.\n");
		if( errno == EINVAL ) fprintf(stderr,"ROBOA: ERREUR: socket(): Protocole demande inconnu.\n");
		exit(EXIT_FAILURE);
	}

	/* On parametre le descripteur de socket pour que l'application rende l'adresse et le port immediatement apres sa fermeture sans duree de
	 *  retention par le noyau, et, que plusieurs sockets puissent s'attacher au meme port (SO_REUSEADDR=1)
	*/
	ParametreSocket=1;
	
	if( setsockopt(IdSocketServeurROBOA,SOL_SOCKET,SO_REUSEADDR,&ParametreSocket,sizeof(int)) == -1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Impossible de parametrer le descripteur de la socket SO_REUSEADDR pour le serveur ROBOA, numero d'erreur %d.\n",errno);
		if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Le descripteur de socket n'est pas valide.\n");
		if( errno == ENOTSOCK ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Le descripteur est un fichier et pas une socket.\n");
		if( errno == ENOPROTOOPT ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Parametre option inconnue pour ce protocole.\n");
		if( errno == EFAULT ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Mauvais pointeur passe en parametre.\n");
		exit(EXIT_FAILURE);
	}

	/* On attache/nomme le point de communication a une adresse:port du domaine parametre (IP:PORT)
	*/
	memset(&AdresseSocketServeur,0,sizeof(struct sockaddr_in));	/* Tous les octets a zero pour initialisation car pas tous parametres     */

	AdresseSocketServeur.sin_family=AF_INET;			/* Famille de socket IP  (man ip)                                         */
	AdresseSocketServeur.sin_addr.s_addr=htonl(INADDR_ANY);		/* Sur n'importe quelle adresse et donc interface de la machine (0.0.0.0) */
									/*  dans l'ordre d'octet reseau (32 bits)                                 */
	AdresseSocketServeur.sin_port=htons(22443);			/* Le numero de port d'ecoute dans l'ordre d'octet reseau (16 bits)       */

	if( bind(IdSocketServeurROBOA,(struct sockaddr *) &AdresseSocketServeur,sizeof(struct sockaddr_in)) == -1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: bind(): Impossible d'attacher la socket a l'adresse:port pour le serveur ROBOA, numero d'erreur %d.\n",errno);
		if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: bind(): Le descripteur de socket n'est pas valide.\n");
		if( errno == EINVAL ) fprintf(stderr,"ROBOA: ERREUR: bind(): La socket possede deja une adresse.\n");
		if( errno == EACCES ) fprintf(stderr,"ROBOA: ERREUR: bind(): L'adresse n'est utilisable qu'en super utilisateur.\n");
		if( errno == ENOTSOCK ) fprintf(stderr,"ROBOA: ERREUR: bind(): Le descripteur est un fichier et pas une socket.\n");
		exit(EXIT_FAILURE);
	}

	/* On place l'application serveur en etat d'ecoute des connexions entrantes sur l'adresse:port definie. On fixe la limite de la liste
	 *  des requetes de connexions entrantes en attente de traitement par l'application a ListeClientsMax.
	*/
	if( listen(IdSocketServeurROBOA,ListeClientsMax) == -1 )
	{
		fprintf(stderr,"ROBOA: ERREUR: listen(): Impossible d'initier l'ecoute des requetes de connexions entrantes sur le serveur ROBOA, numero d'erreur %d.\n",errno);
		if( errno == EADDRINUSE ) fprintf(stderr,"ROBOA: ERREUR: listen(): Une autre socket ecoute sur la meme adresse:port.\n");
		if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: listen(): Le descripteur de socket n'est pas valide.\n");
		if( errno == ENOTSOCK ) fprintf(stderr,"ROBOA: ERREUR: listen(): Le descripteur n'est pas une socket.\n");
		if( errno == EOPNOTSUPP ) fprintf(stderr,"ROBOA: ERREUR: listen(): Ce type de socket ne supporte pas listen().\n");
		exit(EXIT_FAILURE);
	}

	/* On cree la boucle d'ecoute mono-client/mono-processus dont on ne sortira que par l'ordre du client authentifie
	*/
	do
	{
		int IdSocketSession;				/* Descripteur de la socket de session courante                               */
		int AttributsSocketSession;			/* Attributs de controle de la socket de session                              */
		struct sockaddr_in AdresseSocketClient;		/* Adresse de la socket de session du client _in car domaine Internet         */
		socklen_t TailleStructAdresseSocketClient;	/* Nombre d'octets de la structure de donnees de l'adresse du client          */
	
		memset(&AdresseSocketClient,0,sizeof(struct sockaddr_in));	/* Tous les octets de la structure a zero pour initialisation */

		fprintf(stdout,"ROBOA: En attente de connexion d'un client.\n");

		/* On extrait et traite la premiere entree de la liste des requetes de connexions entrantes en attente
		*/
		TailleStructAdresseSocketClient=sizeof(struct sockaddr_in);	/* obligatoire : Le nombre d'octets reserves                  */
		
		if( (IdSocketSession=accept(IdSocketServeurROBOA,(struct sockaddr *) &AdresseSocketClient,&TailleStructAdresseSocketClient)) == -1 )
		{
			fprintf(stderr,"ROBOA: ERREUR: accept(): Impossible de recuperer la premiere entree de la liste des requetes de connexions entrantes sur le serveur ROBOA, numero d'erreur %d.\n",errno);
			if( errno == EAGAIN || errno == EWOULDBLOCK ) fprintf(stderr,"ROBOA: ERREUR: accept(): La socket est en mode non bloquante mais il n'y a aucune requete de connexion dans la liste.\n");
			if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: accept(): Le descripteur de socket n'est pas valide.\n");
			if( errno == ENOTSOCK ) fprintf(stderr,"ROBOA: ERREUR: accept(): Le descripteur n'est pas une socket.\n");
			if( errno == EOPNOTSUPP ) fprintf(stderr,"ROBOA: ERREUR: accept(): La socket de reference n'est pas une SOCK_STREAM.\n");
			if( errno == EINTR ) fprintf(stderr,"ROBOA: ERREUR: accept(): Appel systeme interrompu par l'arrivee d'un signal avant qu'une connexion valide ne survienne.\n");
			if( errno == ECONNABORTED ) fprintf(stderr,"ROBOA: ERREUR: accept(): Une connexion a ete abandonnee.\n");
			if( errno == EINVAL ) fprintf(stderr,"ROBOA: ERREUR: accept(): La socket n'est pas en attente de connexion.\n");
			if( errno == EMFILE ) fprintf(stderr,"ROBOA: ERREUR: accept(): La limite de la table des descripteur est atteinte.\n");
			if( errno == EFAULT ) fprintf(stderr,"ROBOA: ERREUR: accept(): L'adresse n'est pas dans l'espace d'adressage accessible en ecriture.\n");
			if( errno == ENOBUFS || errno == ENOMEM ) fprintf(stderr,"ROBOA: ERREUR: accept(): Pas assez de memoire disponible.\n");
			if( errno == EPROTO ) fprintf(stderr,"ROBOA: ERREUR: accept(): Erreur de protocole.\n");
			if( errno == EPERM ) fprintf(stderr,"ROBOA: ERREUR: accept(): Connexion interdite par le firewall.\n");

			Sortir=TRUE;
		}
		else
		{
			/* Une requete de connexion valide d'un client est a traiter
			*/
			if( TailleStructAdresseSocketClient == sizeof(struct sockaddr_in) )
			{
				fprintf(stdout,"ROBOA: Requete de connexion entrante du client %s(0x%X):%d\n",inet_ntoa(AdresseSocketClient.sin_addr),ntohl(AdresseSocketClient.sin_addr.s_addr),AdresseSocketClient.sin_port);
			}
			else
			{
				fprintf(stdout,"ROBOA: ERREUR: Code a reecrire pour traiter et afficher l'adresse du client apres accept().\n");
				exit(EXIT_FAILURE);
			}

			/* Comme le serveur est mono-client on filtre l'adresse du client deja a ce niveau sans initier le SSL
			*/
			if( ntohl(AdresseSocketClient.sin_addr.s_addr) == IPOkClient )
			{
				int RetourES_SSL;		/* Valeur retournee par les fonctions E/S SSL           */
				int TentativesSSL_accept=0;	/* Nombre de tentatives de lecture d'initiation TLS/SSL */
				BIO *SocketSSL_BIO;		/* Pointeur vers une socket SSL dans un objet BIO       */
				SSL *StructSSLServeurROBOA;	/* Structure SSL de connexion                           */
				
				/* Ce client est autorise, on peut initier le SSL
				*/

				/* Parametrage de la socket en mode non bloquant pour gerer un timeout sur SSL_accept()
				*/
				AttributsSocketSession=fcntl(IdSocketSession,F_GETFL);
				fcntl(IdSocketSession,F_SETFL,AttributsSocketSession | O_NONBLOCK);

				/* On cree une socket SSL dans un objet BIO a partir de la socket (normale) de session courante
				 *  BIO_NOCLOSE pour que BIO_free ne ferme pas et ne detruise pas automatiquement la socket
				*/
				if( (SocketSSL_BIO=BIO_new_socket(IdSocketSession,BIO_NOCLOSE)) == NULL )
				{
					fprintf(stderr,"ROBOA: ERREUR: BIO_new_socket(): Impossible de creer une socket SSL dans un objet BIO a partir de la socket (normale) de session courante.\n");
					AfficheErreurPileLibrairieSSL();
					close(IdSocketSession);
				}

				/* Creation de la structure SSL de connexion a partir du contexte ROBOA
				*/
				if( (StructSSLServeurROBOA=SSL_new(ContexteSSLROBOA)) == NULL )
				{
					fprintf(stderr,"ROBOA: ERREUR: SSL_new(): Impossible de creer la structure SSL de connexion a partir du contexte du serveur ROBOA.\n");
					AfficheErreurPileLibrairieSSL();
					close(IdSocketSession);
				}

				/* Si la socket BIO et la structure de connexion SSL sont crees
				*/
				if( SocketSSL_BIO != NULL && StructSSLServeurROBOA != NULL )
				{
					int NegociationInitiee=FALSE;	/* Etat de la negociation TLS/SSL                         */

					/* On prepare la structure de connexion SSL a etre en mode serveur
					*/
					SSL_set_accept_state(StructSSLServeurROBOA);

					/* On connecte l'objet BIO a la structure de connexion SSL pour l'operation de negociation
					 * (OBLIGATOIRE pour SSL_accept())
					 * StructSSLServeurROBOA ne devra plus etre libere par SSL_free() mais par BIO_free(SocketSSL_BIO)
					*/
					SSL_set_bio(StructSSLServeurROBOA,SocketSSL_BIO,SocketSSL_BIO);

					/* On attend que le client initie la negociation TLS/SSL
					*/
					do
					{
						BIO *ConnexionSSL=NULL;		/* Objet BIO de type SSL qui sera chaine a BIO de type buffer    */
						BIO *ConnexionBuff=NULL;	/* Objet BIO de type buffer                                      */
						struct timeval timeout_sock_put;	/* TimeOut socket pour l'emission                        */
						struct timeval timeout_sock_get;	/* TimeOut socket pour la reception                      */
						
						switch( (RetourES_SSL=SSL_accept(StructSSLServeurROBOA)) )
						{
							case 0:
								/* La negociation TLS/SSL n'a pas aboutie mais l'arret est controle par le protocole
								*/
								fprintf(stderr,"ROBOA: ERREUR: SSL_accept(): Erreur lors de la negociation TLS/SSL.\n");
								AfficheErreurES_SSL(StructSSLServeurROBOA,RetourES_SSL);
								/* SSL_free(StructSSLServeurROBOA); */
								if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
								close(IdSocketSession);
								break;

							case 1:
								/* La negociation est inititiee
								*/
								NegociationInitiee=TRUE;

								/* La socket de session repasse dans le mode bloquant pour la suite des echanges
								*/
								AttributsSocketSession=fcntl(IdSocketSession,F_GETFL);
								fcntl(IdSocketSession,F_SETFL,AttributsSocketSession & (~O_NONBLOCK));

								/* On fixe un timeout pour l'emission
								*/
								timeout_sock_put.tv_sec=TIMEOUT_SOCKET_PUT;
								timeout_sock_put.tv_usec=0;

								if( setsockopt(IdSocketSession,SOL_SOCKET,SO_SNDTIMEO,(void *) &timeout_sock_put,sizeof(struct timeval)) == -1 )
								{
									fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Impossible de parametrer SO_SNDTIMEO sur ce systeme, numero d'erreur %d.\n",errno);
									if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Le descripteur de socket n'est pas valide.\n");
									if( errno == ENOPROTOOPT ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): L'option est inconnue pour ce protocole.\n");
								}

								/* On fixe un timeout pour la reception
								*/
								timeout_sock_get.tv_sec=TIMEOUT_SOCKET_GET;
								timeout_sock_get.tv_usec=0;

								if( setsockopt(IdSocketSession,SOL_SOCKET,SO_RCVTIMEO,(void *) &timeout_sock_get,sizeof(struct timeval)) == -1 )
								{
									fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Impossible de parametrer SO_RCVTIMEO sur ce systeme, numero d'erreur %d.\n",errno);
									if( errno == EBADF ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): Le descripteur de socket n'est pas valide.\n");
									if( errno == ENOPROTOOPT ) fprintf(stderr,"ROBOA: ERREUR: setsockopt(): L'option est inconnue pour ce protocole.\n");
								}
								
								
								/* La negociation est reussie entre le serveur et le client selon le contexte defini
								*/
								fprintf(stdout,"ROBOA: Connexion SSL acceptee avec le client %s(0x%X):%d\n",inet_ntoa(AdresseSocketClient.sin_addr),ntohl(AdresseSocketClient.sin_addr.s_addr),AdresseSocketClient.sin_port);

								/* On cree un objet BIO de type SSL avec la methode BIO_f_ssl() qui permet de le specialiser
								*/
								if( (ConnexionSSL=BIO_new(BIO_f_ssl())) == NULL )
								{
									fprintf(stderr,"ROBOA: ERREUR: BIO_new(): Impossible de creer l'objet de connexion BIO de type SSL.\n");
									/* SSL_free(StructSSLServeurROBOA); */
									if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
									close(IdSocketSession);
								}
								else
								{
									/* On attache la structure de connexion SSL a l'objet BIO SSL
									*/
									BIO_set_ssl(ConnexionSSL,StructSSLServeurROBOA,BIO_CLOSE);
					
									/* On cree un objet BIO avec buffer avec la methode BIO_f_buffer() qui permet de le specialiser,
								 	* le tableau est de DEFAULT_BUFFER_SIZE octets par defaut
									*/
									if( (ConnexionBuff=BIO_new(BIO_f_buffer())) == NULL )
									{
										fprintf(stderr,"ROBOA: ERREUR: BIO_new(): Impossible de creer l'objet de connexion BIO de type buffer.\n");
										if( !BIO_free(ConnexionSSL) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer ConnexionSSL.\n");
										/* SSL_free(StructSSLServeurROBOA); */
										if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
										close(IdSocketSession);
									}
									else
									{
										int BoucleCommande=TRUE;/* Drapeau pour savoir si la boucle d'attente des commandes est valide */
										int SesameOuvreToi=FALSE;  /* Chaine mot de passe pour Login et accepter les commandes clients */
										int NombreShutdown=0;	/* Nombre de tentatives possibles de fermeture de la connexion SSL     */
										int RetourGets;		/* Code retour de BIO_gets() */
										unsigned long CompteurEnAttente=0;	/* Compteur du nombre de commandes en attentes recues  */
										static char ChaineGets[TAILLE_MAXI_CHAINE_BIO];	/* Chaine pour BIO_gets()                      */
										static char ChainePuts[TAILLE_MAXI_CHAINE_BIO];	/* Chaine pour BIO_puts()                      */
										const SSL_CIPHER *Chiffreur;	/* Pointeur vers description du chiffreur de la connexion              */
										struct sigaction AncienGestSignalSIGPIPE;	/* Ancien gestionnaire du signal SIGPIPE       */
										struct sigaction NouvGestSignalSIGPIPE;		/* Nouveau gestionnaire du signal SIGPIPE      */

										/* On chaine l'objet BIO SSL avec l'objet BIO buffer
										 * La chaine d'objets est donc :
										 *  Serveur:ConnexionBuff->ConnexionSSL->StructSSLServeurROBOA->IdSocketSession -> Client
										*/
										BIO_push(ConnexionBuff,ConnexionSSL);

										/* On parametre le nouveau gestionnaire du signal SIGPIPE
										 * SIGPIPE est declanche lorsqu'un processus tente d'ecrire dans tube ferme
										*/
										NouvGestSignalSIGPIPE.sa_handler=FnHandlerSIGPIPE;

										if( sigaction(SIGPIPE,&NouvGestSignalSIGPIPE,&AncienGestSignalSIGPIPE) < 0 )
										{
											fprintf(stderr,"ROBOA: ERREUR: sigaction(): Impossible de parametrer un gestionnaire du signal SIGPIPE.\n");
										}
										/* kill(getpid(),SIGPIPE); pour tester */

										/* On envoie le message de bienvenue au client identifie
										*/
										Chiffreur=SSL_get_current_cipher(StructSSLServeurROBOA);

										sprintf(ChainePuts,"%s (Chiffrement:%s, Protocoles:%s)\n",MessageBienvenue,SSL_CIPHER_get_name(Chiffreur),SSL_CIPHER_get_version(Chiffreur));

										if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

										/* Boucle d'attente des commandes du client
										*/
										while( BoucleCommande )
										{
											if( (RetourGets=BIO_gets(ConnexionBuff,ChaineGets,TAILLE_MAXI_CHAINE_BIO)) <= 0 )
											{
												if( RetourGets == 0 ) fprintf(stderr,"ROBOA: ERREUR: BIO_gets(): Connexion abandonnee par le client.\n");
												if( RetourGets == -1 ) fprintf(stderr,"ROBOA: ERREUR TIMEOUT: BIO_gets(): Duree de non activite depassee, la connexion est consideree comme perdue.\n");
												BoucleCommande=FALSE;
											}
											else
											{
												int i=0;		/* Variable indice          */
												int IdCmdClient;	/* Id de commande client    */
												int Axe;		/* Variable numero axe      */
												i32 PositionCodeur1;	/* Variable position axe    */
												i32 PositionCodeur2;
												i32 PositionCodeur3;
												i32 PositionCodeur4;
												i32 PositionMot;	/* Variable position moteur */
			

												/* On coupe la chaine au premier \r ou \n
												*/
												while( ChaineGets[i] != 0 )
												{
													if( ChaineGets[i] == '\r' )
													{
														ChaineGets[i]=0;
														break;
													}
													if( ChaineGets[i] == '\n' )
													{
														ChaineGets[i]=0;
														break;
													}

													i++;
												}

												if( SesameOuvreToi ) fprintf(stdout,"ROBOA: S<-C %4d: %s\n",RetourGets,ChaineGets);

												/* Recherche de la commande dans la liste
												*/
												for( IdCmdClient=0; ListeCmdClientROBOA[IdCmdClient][0] != 0; IdCmdClient++ ) if( RetourGets > 2 ) if( ListeCmdClientROBOA[IdCmdClient][1] == ChaineGets[1] ) if( strncmp(ListeCmdClientROBOA[IdCmdClient],ChaineGets,strlen(ListeCmdClientROBOA[IdCmdClient])) == 0 ) break;
												
												/* Si la chaine login/mdp sesame n'a pas ete recue
												*/
												if( !SesameOuvreToi )
												{
													if( IdCmdClient == ROBOA_CMD_SESAMEOUVRETOI )
													{
														SesameOuvreToi=TRUE;
													}
													
													IdCmdClient=-1;
												}
												
												/* Selon l'identifieur de la commande
												*/
												switch( IdCmdClient )
												{
													case ROBOA_CMD_ARRET:
														/*
														 * Arret et sortie du logiciel
														*/
														if( !ArretDouxDesactivationAxes(ConnexionBuff) )
														{
															sprintf(ChainePuts,"ROBOA: ERREUR: Impossible d'arreter et desactiver les axes.\n");
															EnvoyerChaineBIO(ConnexionBuff,ChainePuts);
														}
														BoucleCommande=FALSE;
														Sortir=TRUE;
														ARRET_Machine=TRUE;
														break;
														
													case ROBOA_CMD_DECONNEXION:
														/*
														 * Deconnexion du client
														*/
														BoucleCommande=FALSE;
														break;

													case ROBOA_CMD_EN_ATTENTE:
														/*
														 * Serveur en attente, en vie ?
														*/
														CompteurEnAttente++;

														sprintf(ChainePuts,"%s%lu\n",ListeRepServeurROBOA[ROBOA_REP_EN_ATTENTE],CompteurEnAttente);

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														break;

													case ROBOA_CMD_INITCONFIGHW:
														/*
														 * Initialisation et configuration de la carte de controle des mouvements National Instruments PCI7334 et MID7604
														*/
														if( !InitialiseConfigureCarteMouvement(ConnexionBuff) )
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONINIT]);
														}
														else
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_OKINIT]);
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;
														
														break;

													case ROBOA_CMD_RECH_INDEX_AXE:
														/*
														 * Recherche de l'index codeur d'un axe
														*/
														if( sscanf(ChaineGets+strlen(ListeCmdClientROBOA[IdCmdClient]),"%d",&Axe) != 1 )
														{
															sprintf(ChainePuts,"%s(%s)\n",ListeRepServeurROBOA[ROBOA_REP_NONOKINDEXAXE],ChaineGets);
														}
														else
														{
															int retour;	/* valeur retour */
															
															switch( Axe )
															{
																case 1: retour=RechercherIndexCodeur(NIMC_AXIS1,TIMEOUT_RECH_INDEX_CODEUR,ConnexionBuff); break;
																case 2: retour=RechercherIndexCodeur(NIMC_AXIS2,TIMEOUT_RECH_INDEX_CODEUR,ConnexionBuff); break;
																case 3: retour=RechercherIndexCodeur(NIMC_AXIS3,TIMEOUT_RECH_INDEX_CODEUR,ConnexionBuff); break;
																case 4: retour=RechercherIndexCodeur(NIMC_AXIS4,TIMEOUT_RECH_INDEX_CODEUR,ConnexionBuff); break;

																default:
																	retour=FALSE;
																	break;
															}

															if( !retour )
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_NONOKINDEXAXE],Axe);
															}
															else
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_OKINDEXAXE],Axe);
															}
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);

														break;
														
													case ROBOA_CMD_RECH_POSREPOS_AXE:
														/*
														 * Recherche position de repos de l'axe
														*/
														if( sscanf(ChaineGets+strlen(ListeCmdClientROBOA[IdCmdClient]),"%d",&Axe) != 1 )
														{
															sprintf(ChainePuts,"%s(%s)\n",ListeRepServeurROBOA[ROBOA_REP_NONOKPOSREPOSAXE],ChaineGets);
														}
														else
														{
															int retour;	/* valeur retour */
															
															switch( Axe )
															{
																case 1: retour=RechercherPositionRepos(NIMC_AXIS1,TIMEOUT_RECH_POSITION_REPOS,ConnexionBuff); break;
																case 2: retour=RechercherPositionRepos(NIMC_AXIS2,TIMEOUT_RECH_POSITION_REPOS,ConnexionBuff); break;
																case 3: retour=RechercherPositionRepos(NIMC_AXIS3,TIMEOUT_RECH_POSITION_REPOS,ConnexionBuff); break;
																case 4: retour=RechercherPositionRepos(NIMC_AXIS4,TIMEOUT_RECH_POSITION_REPOS,ConnexionBuff); break;

																default:
																	retour=FALSE;
																	break;
															}

															if( !retour )
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_NONOKPOSREPOSAXE],Axe);
															}
															else
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_OKPOSREPOSAXE],Axe);
															}
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);

														break;
														
													case ROBOA_CMD_POSAXESPASENC:
														/*
														 * Demande de la postions des axes
														 *  en pas encodeurs en quadrature
														*/
														if( !PositionsAxesEnPasCodeurQuadrature(&PositionCodeur1,&PositionCodeur2,&PositionCodeur3,&PositionCodeur4,ConnexionBuff) )
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONPOSAXESPASENC]);
														}
														else
														{
															sprintf(ChainePuts,"%s %ld %ld %ld %ld\n",ListeRepServeurROBOA[ROBOA_REP_POSAXESPASENC],PositionCodeur1,PositionCodeur2,PositionCodeur3,PositionCodeur4);
														}

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														break;

													case ROBOA_CMD_POSAXESPASMOT:
														/*
														 * Demande de la position des axes
														 *  en pas moteur
														*/
														if( !PositionsAxesEnPasMoteur(&PositionCodeur1,&PositionCodeur2,&PositionCodeur3,&PositionCodeur4,ConnexionBuff) )
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONPOSAXESPASMOT]);
														}
														else
														{
															sprintf(ChainePuts,"%s %ld %ld %ld %ld\n",ListeRepServeurROBOA[ROBOA_REP_POSAXESPASMOT],PositionCodeur1,PositionCodeur2,PositionCodeur3,PositionCodeur4);
														}

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														break;

													case ROBOA_CMD_MVTAXEPASMOT:
														/*
														 * Demande de mouvement d'un axe
														 *  en pas moteur
														*/
														if( sscanf(ChaineGets+strlen(ListeCmdClientROBOA[IdCmdClient]),"%d %ld",&Axe,&PositionMot) != 2 )
														{
															sprintf(ChainePuts,"%s(%s)\n",ListeRepServeurROBOA[ROBOA_REP_NONOKMVTAXEPASMOT],ChaineGets);
														}
														else
														{
															int retour;	/* Valeur retour */
															
															switch( Axe )
															{
																case 1: retour=PositionnerAxe(NIMC_AXIS1,PositionMot); break;
																case 2: retour=PositionnerAxe(NIMC_AXIS2,PositionMot); break;
																case 3: retour=PositionnerAxe(NIMC_AXIS3,PositionMot); break;
																case 4: retour=PositionnerAxe(NIMC_AXIS4,PositionMot); break;
																default:
																	retour=FALSE;
																	break;
															}
															
															if( !retour )
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_NONOKMVTAXEPASMOT],Axe);
															}
															else
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_OKMVTAXEPASMOT],Axe);
															}
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;
														
														AffichagePurgeErreursModales(ConnexionBuff);
														
														break;

													case ROBOA_CMD_DEMARRERAXE:
														/*
														 * Demande de demarrage du mvt d'un axe
														*/
														if( sscanf(ChaineGets+strlen(ListeCmdClientROBOA[IdCmdClient]),"%d",&Axe) != 1 )
														{
															sprintf(ChainePuts,"%s(%s)\n",ListeRepServeurROBOA[ROBOA_REP_NONOKDEMARRERAXE],ChaineGets);
														}
														else
														{
															switch( Axe )
															{
																case 1: CodeRetourFlex=flex_start(IdCarte,NIMC_AXIS1,0); break;
																case 2: CodeRetourFlex=flex_start(IdCarte,NIMC_AXIS2,0); break;
																case 3: CodeRetourFlex=flex_start(IdCarte,NIMC_AXIS3,0); break;
																case 4: CodeRetourFlex=flex_start(IdCarte,NIMC_AXIS4,0); break;
																	
																default:
																	CodeRetourFlex=~NIMC_noError;
																	break;
															}

															if( CodeRetourFlex != NIMC_noError )
															{
																if( Axe >= 1 && Axe <= 4 )
																{
																	CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
																	sprintf(ChainePuts,"ROBOA: ERREUR: flex_start(): Impossible de demarrer l'axe %d: %s.\n",Axe,ChaineErreurNonModale);
																	
																	fprintf(stderr,ChainePuts);
																
																	BIO_printf(ConnexionBuff,ChainePuts);
																}
																
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_NONOKDEMARRERAXE],Axe);
															}
															else
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_OKDEMARRERAXE],Axe);
															}
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);

														break;

													case ROBOA_CMD_DEMARRERTOUSAXES:
														/*
														 * Demande de demarrage du mvt de tous
														 *  les axes
														*/
														CodeRetourFlex=flex_start(IdCarte,0,0x1E);
														
														if( CodeRetourFlex != NIMC_noError )
														{
															CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
															sprintf(ChainePuts,"ROBOA: ERREUR: flex_start(): Impossible de demarrer tous les axes: %s.\n",ChaineErreurNonModale);
															
															fprintf(stderr,ChainePuts);
																
															BIO_printf(ConnexionBuff,ChainePuts);
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONOKDEMARRERAXES]);
														}
														else
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_OKDEMARRERAXES]);
														}

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);
														
														break;

													case ROBOA_CMD_ARRETAXE:
														/*
														 * Demande de l'arret du mvt d'un axe
														*/
														if( sscanf(ChaineGets+strlen(ListeCmdClientROBOA[IdCmdClient]),"%d",&Axe) != 1 )
														{
															sprintf(ChainePuts,"%s(%s)\n",ListeRepServeurROBOA[ROBOA_REP_NONOKARRETAXE],ChaineGets);
														}
														else
														{
															switch( Axe )
															{
																case 1: CodeRetourFlex=flex_stop_motion(IdCarte,NIMC_AXIS1,NIMC_DECEL_STOP,0); break;
																case 2: CodeRetourFlex=flex_stop_motion(IdCarte,NIMC_AXIS2,NIMC_DECEL_STOP,0); break;
																case 3: CodeRetourFlex=flex_stop_motion(IdCarte,NIMC_AXIS3,NIMC_DECEL_STOP,0); break;
																case 4: CodeRetourFlex=flex_stop_motion(IdCarte,NIMC_AXIS4,NIMC_DECEL_STOP,0); break;
																	
																default:
																	CodeRetourFlex=~NIMC_noError;
																	break;
															}

															if( CodeRetourFlex != NIMC_noError )
															{
																if( Axe >= 1 && Axe <= 4 )
																{
																	CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
																	sprintf(ChainePuts,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement de l'axe %d: %s.\n",Axe,ChaineErreurNonModale);
																	
																	fprintf(stderr,ChainePuts);
																
																	BIO_printf(ConnexionBuff,ChainePuts);
																}

																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_NONOKARRETAXE],Axe);
															}
															else
															{
																sprintf(ChainePuts,"%s%d\n",ListeRepServeurROBOA[ROBOA_REP_OKARRETAXE],Axe);
															}
														}
														
														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;
														
														AffichagePurgeErreursModales(ConnexionBuff);
														
														break;

													case ROBOA_CMD_ARRETTOUSAXES:
														/*
														 * Demande d'arret du mvt de tous
														 *  les axes
														*/
														CodeRetourFlex=flex_stop_motion(IdCarte,0,NIMC_DECEL_STOP,0x1E);
														
														if( CodeRetourFlex != NIMC_noError )
														{
															CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
															sprintf(ChainePuts,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement des axes: %s.\n",ChaineErreurNonModale);
															
															fprintf(stderr,ChainePuts);
																
															BIO_printf(ConnexionBuff,ChainePuts);
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONOKARRETAXES]);
														}
														else
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_OKARRETAXES]);
														}

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);

														break;

													case ROBOA_CMD_ARRETTOUSAXESURGENT:
														/*
														 * Demande d'arret URGENT du mvt de tous
														 *  les axes
														*/
														CodeRetourFlex=flex_stop_motion(IdCarte,0,NIMC_HALT_STOP,0x1E);
														
														if( CodeRetourFlex != NIMC_noError )
														{
															CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
															sprintf(ChainePuts,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement des axes en urgence: %s.\n",ChaineErreurNonModale);
															
															fprintf(stderr,ChainePuts);
																
															BIO_printf(ConnexionBuff,ChainePuts);
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_NONOKARRETAXESURGENT]);
														}
														else
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_OKARRETAXESURGENT]);
														}

														if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;

														AffichagePurgeErreursModales(ConnexionBuff);

														break;


													default:
														/* Si la chaine login/mdp sesame est recue
														*/
														if( SesameOuvreToi && IdCmdClient != -1 )
														{
															sprintf(ChainePuts,"%s\n",ListeRepServeurROBOA[ROBOA_REP_CMD_INCONNUE]);
															if( EnvoyerChaineBIO(ConnexionBuff,ChainePuts) <= 0 ) BoucleCommande=FALSE;
														}
														
														break;
												}
											}
										}


										/* On passe la connexion SSL en mode de fermeture en toute quietude
										 *  la notification de fermeture n'est pas envoye au paire pour prevenir
										 *  le crash de SSL_shutdown() en cas de perte de connexion ou disparition
										 *  du paire
										*/
										SSL_set_quiet_shutdown(StructSSLServeurROBOA,1);

										/* On ferme la connexion SSL
										*/
										while( (RetourES_SSL=SSL_shutdown(StructSSLServeurROBOA)) != 1 && NombreShutdown < 4 )
										{
											NombreShutdown++;
	
											if( RetourES_SSL == 0 )
											{
												fprintf(stderr,"ROBOA: ERREUR: SSL_shutdown(): Tentative %d: La fermeture de la socket SSL n'est pas terminee.\n",NombreShutdown);
												/* On signale au client que la socket ne sera plus utilisee pour l'ecriture
												 *  ce qui evite de boucler plusieurs
												*/
												if( shutdown(IdSocketSession,SHUT_WR) < 0 )
												{
													fprintf(stderr,"ROBOA: ERREUR: shutdown(): Impossible de signaler au client que la socket ne sera plus utilisee pour l'ecriture.\n");
												}
											}
											else
											{
												fprintf(stderr,"ROBOA: ERREUR: SSL_shutdown(): Impossible de fermer la connexion SSL, erreur fatale au niveau du protocole ou erreur fatale de connexion.\n");
												AfficheErreurES_SSL(StructSSLServeurROBOA,RetourES_SSL);
											}
										}
					
										if( RetourES_SSL == 1 )
										{
											fprintf(stdout,"ROBOA: Connexion SSL fermee avec le client %s(0x%X):%d\n\n",inet_ntoa(AdresseSocketClient.sin_addr),ntohl(AdresseSocketClient.sin_addr.s_addr),AdresseSocketClient.sin_port);
										}
										else
										{
											fprintf(stderr,"ROBOA: ERREUR: SSL_shutdown(): Impossible de fermer la connexion SSL apres plusieurs tentatives.\n");
										}

										/* On parametre le gestionnaire de SIGPIPE initial
										*/
										if( sigaction(SIGPIPE,&AncienGestSignalSIGPIPE,NULL) < 0 )
										{
											fprintf(stderr,"ROBOA: ERREUR: sigaction(): Impossible de parametrer un gestionnaire du signal SIGPIPE.\n");
										}

										/* Liberation de toute la chaine BIO
										*/
										if( !BIO_free(ConnexionBuff) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer ConnexionBuff.\n");
										if( !BIO_free(ConnexionSSL) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer ConnexionSSL.\n");
										/* SSL_free(StructSSLServeurROBOA); */
										if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
										close(IdSocketSession);
									}
								}
								
								break;
								
							case -1:
								/* SSL_accept() est en mode non bloquant et son appel retourne SSL_ERROR_WANT_READ
								 *  car la negociation SSL/TLS n'a pas demarree
								*/
								TentativesSSL_accept++;
								
								fprintf(stdout,"ROBOA: La negociation TLS/SSL n'est pas encore initiee par le client n=%d.\n",TentativesSSL_accept);
								/* On va patienter avant de relancer un appel SSL_accept()
								*/
								sleep(1);
								
								break;
						}

					} while( TentativesSSL_accept < TimeoutSSLAccept && !NegociationInitiee );

					/* Si aucune negociation TLS/SSL n'a ete inititiee et que le timeout est depasse
					*/
					if( TentativesSSL_accept == TimeoutSSLAccept && !NegociationInitiee )
					{
						fprintf(stderr,"ROBOA: ERREUR: Aucune negociation TLS/SSL n'a ete initiee par le client.\n");
						/* SSL_free(StructSSLServeurROBOA); */
						if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
						close(IdSocketSession);
					}
				}
				else
				{
					/* Un des deux ou les deux objets n'ont pus etre crees
					*/
					if( StructSSLServeurROBOA != NULL && SocketSSL_BIO == NULL ) SSL_free(StructSSLServeurROBOA);
					if( SocketSSL_BIO != NULL ) if( !BIO_free(SocketSSL_BIO) ) fprintf(stderr,"ROBOA: ERREUR: BIO_free(): Impossible de liberer SocketSSL_BIO.\n");
				}
			}
			else
			{
				/* Ce client n'est pas autorise, on ferme la requete de connexion et on en attend une autre
				*/
				fprintf(stdout,"ROBOA: Ce client n'est pas autorise, connexion refusee.\n");

				close(IdSocketSession);
			}
		}
		
	} while( !Sortir );
	
	
	/* Si il y en a, affichage et purge des erreurs modales
	*/
	while( TestRecupCompChaineErreurFlexMotionModale(&IdCommandeErr,&IdRessourceErr,&CodeErreurModaleErr,ChaineErreurModale) ) fprintf(stderr,"ROBOA: ERREUR MODALE: %s.\n",ChaineErreurModale);

	
	/* Liberation de la memoire
	*/
	DH_free(ParametresDH);
	SSL_CTX_free(ContexteSSLROBOA);
	ERR_free_strings();


	/* Si on doit arreter la machine
	*/
	if( ARRET_Machine )
	{
		system("/bin/sync ; /bin/sleep 60s ; /usr/bin/sudo /sbin/halt");
	}


	exit(EXIT_SUCCESS);
}


/* FONCTION D'EMISSION D'UNE CHAINE PAR UN OBJET BIO ET TRAITEMENT DU CODE RETOUR
 *
 * CE:	On passe un pointeur sur l'objet BIO ;
 *
 * 	On passe un pointeur vers la chaine a envoyer ;
 *
 * CS:	La fonction retourne la valeur retournee par BIO_puts()
*/

int EnvoyerChaineBIO(BIO *comm,char *chaine)
{
	int errno_avant=errno;		/* Sauvegarde de errno           */
	int RetourPuts;			/* Valeur retournee par BIO_puts */

	/* Emission de la chaine sur l'objet BIO bufferise
	*/
	RetourPuts=BIO_puts(comm,chaine);
	
	BIO_flush(comm);

	if( RetourPuts <= 0 || errno != errno_avant )
	{
		if( RetourPuts == 0 ) fprintf(stderr,"ROBOA: ERREUR: BIO_puts(): Connexion abandonnee par le client.\n");
		
		if( RetourPuts < 0 ) fprintf(stderr,"ROBOA: ERREUR TIMEOUT: BIO_puts(): Duree de non activite depassee, la connexion est consideree comme perdue.\n");
		
		if( errno != errno_avant )
		{
			RetourPuts=-1;		/* Pour detecter l'erreur simplement avec le code retour */
			
			fprintf(stderr,"ROBOA: ERREUR: BIO_puts(): errno=%d : %s.\n",errno,strerror(errno));
		}
	}
	else
	{
		fprintf(stdout,"ROBOA: S->C %4d: %s",RetourPuts,chaine);
	}

	return RetourPuts;
}


/* FONCTION D'INITIALISATION ET DE CONFIGURATION DE LA CARTE DE MOUVEMENT PCI7334
 *
 * CE:	On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *
 * CS:	La fonction est vraie si aucun probleme n'est detecte lors de l'initialisation et la configuration
*/

int InitialiseConfigureCarteMouvement(BIO *comm)
{
	int RetourTestErreurModale;	/* Valeur de retour de la fonction de test des erreurs modales                         */
	int ErreurModale=FALSE;		/* Indicateur d'une erreur modale                                                      */
	u16 CSR;			/* Registre Communications Status Register                                             */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreur non modale         */
	u16 IdCommandeErr;					/* Numero de commande d'une erreur modale                      */
	u16 IdRessourceErr;					/* Numero de ressource d'une erreur modale                     */
	i32 CodeErreurModaleErr;				/* Code d'erreur d'une erreur modale                           */
	char ChaineErreurModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur modale                      */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */

	
	/* On commence par laver le bit de mise sous tension de l'interface et demarrer le controleur
	 * pour qu'il soit pret a accepter des fonctions. A la mise sous tension le controleur est
	 * suspendu dans un etat Power-Up et un bit est positionne dans le Communication Status Register
	 * C.S.R. Cette fonction permet de laver ce bit et prepare le controleur pour des communications
	 * de controle de mouvements.
	*/
	if( (CodeRetourFlex=flex_clear_pu_status(IdCarte)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_clear_pu_status(): Impossible de laver le bit PowerUp: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	/* On attend que le bit "PowerUp" passe au niveau bas
	*/
	do
	{
		if( (CodeRetourFlex=flex_read_csr_rtn(IdCarte,&CSR)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_read_csr_rtn(): %s.\n",ChaineErreurNonModale);
			return FALSE;
		}

		/* BIO_printf(comm,"ROBOA: Test bit PowerUp(%u): CSR=%u\n",NIMC_POWER_UP_RESET,CSR); */
			
	} while( (CSR & NIMC_POWER_UP_RESET) );


	/* Pour pouvoir configurer un axe, il faut qu'ils soient tous inoperants (disable)
	*/
	if( (CodeRetourFlex=flex_enable_axes(IdCarte,0,0,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_enable_axes(): Impossible de desactiver des axes pour pouvoir les configurer: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	
	/* Configuration du mode de fonctionnement des axes : axe n  associe au moteur n et l'encodeur n
	*/
	if( (CodeRetourFlex=flex_config_axis(IdCarte,NIMC_AXIS1,NIMC_ENCODER1,0,NIMC_STEP_OUTPUT1,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_axis(): Impossible de configurer l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_axis(IdCarte,NIMC_AXIS2,NIMC_ENCODER2,0,NIMC_STEP_OUTPUT2,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_axis(): Impossible de configurer l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_axis(IdCarte,NIMC_AXIS3,NIMC_ENCODER3,0,NIMC_STEP_OUTPUT3,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_axis(): Impossible de configurer l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_axis(IdCarte,NIMC_AXIS4,NIMC_ENCODER4,0,NIMC_STEP_OUTPUT4,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_axis(): Impossible de configurer l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	
	/* Comme les axes sont configures, on peut maintenant les permettre (il ne s'agit pas d'une mise en courant mais juste d'une validation
	    pour le controleur)
	*/
	if( (CodeRetourFlex=flex_enable_axes(IdCarte,NIMC_AXIS_CTRL,NIMC_PID_RATE_250,0x1E)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_enable_axes(): Impossible d'activer les axes du controleur: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration du nombre d'impulsions par tour des moteurs pas a pas (attention depend du reglage du nombre de micropas sur le MID)
	*/
	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS1,NIMC_STEPS,NbPasMoteurAxe1*NbMPAxe1)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre d'impulsions par tour pour le moteur de l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS2,NIMC_STEPS,NbPasMoteurAxe2*NbMPAxe2)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre d'impulsions par tour pour le moteur de l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS3,NIMC_STEPS,NbPasMoteurAxe3*NbMPAxe3)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre d'impulsions par tour pour le moteur de l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS4,NIMC_STEPS,NbPasMoteurAxe4*NbMPAxe4)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre d'impulsions par tour pour le moteur de l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	
	/* Configuration du nombre de pas en quadrature des codeurs incrementaux utilises = 4 * Nombre de pas codeurs
	*/
	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS1,NIMC_COUNTS,4*NbPasCodeurAxe1)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre de pas en quadrature du codeur 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS2,NIMC_COUNTS,4*NbPasCodeurAxe2)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre de pas en quadrature du codeur 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS3,NIMC_COUNTS,4*NbPasCodeurAxe3)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre de pas en quadrature du codeur 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	if( (CodeRetourFlex=flex_load_counts_steps_rev(IdCarte,NIMC_AXIS4,NIMC_COUNTS,4*NbPasCodeurAxe4)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_counts_steps_rev(): Impossible de configurer le nombre de pas en quadrature du codeur 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration du mode industriel "Step & Direction" (ou "CW & CCW")(bit 2) et de la polarite (sens de rotation du moteur pour +)(bit 0)
	    des sorties des moteurs pas a pas
	    Avec le bit 0 on peut changer le sens de rotation du moteur sans modifier le branchement electrique.
	*/
	if( (CodeRetourFlex=flex_config_step_mode_pol(IdCarte,NIMC_AXIS1,0x05)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_step_mode_pol(): Impossible de configurer le mode industriel et la polarite de la sortie du moteur pas a pas de l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_step_mode_pol(IdCarte,NIMC_AXIS2,0x05)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_step_mode_pol(): Impossible de configurer le mode industriel et la polarite de la sortie du moteur pas a pas de l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_step_mode_pol(IdCarte,NIMC_AXIS3,0x05)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_step_mode_pol(): Impossible de configurer le mode industriel et la polarite de la sortie du moteur pas a pas de l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_config_step_mode_pol(IdCarte,NIMC_AXIS4,0x05)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_config_step_mode_pol(): Impossible de configurer le mode industriel et la polarite de la sortie du moteur pas a pas de l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration de la polarite des entrees rapides des codeurs incrementaux
	*/
/*	if( (CodeRetourFlex=flex_set_hs_cap_pol(IdCarte,0,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_hs_cap_pol(): Impossible de configurer la polarite des codeurs incrementaux: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
*/

	
	/* Configuration du mode de boucle utilise pour chaque axe
	*/
	if( (CodeRetourFlex=flex_set_stepper_loop_mode(IdCarte,NIMC_AXIS1,/*NIMC_OPEN_LOOP*/ NIMC_CLOSED_LOOP)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_stepper_loop_mode(): Impossible de configurer le type de boucle pour l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_set_stepper_loop_mode(IdCarte,NIMC_AXIS2,NIMC_OPEN_LOOP /*NIMC_CLOSED_LOOP*/)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_stepper_loop_mode(): Impossible de configurer le type de boucle pour l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_set_stepper_loop_mode(IdCarte,NIMC_AXIS3,NIMC_OPEN_LOOP /*NIMC_CLOSED_LOOP*/)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_stepper_loop_mode(): Impossible de configurer le type de boucle pour l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_set_stepper_loop_mode(IdCarte,NIMC_AXIS4,NIMC_OPEN_LOOP /*NIMC_CLOSED_LOOP*/)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_stepper_loop_mode(): Impossible de configurer le type de boucle pour l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration de la vitesse maximale de chaque axe en tours par minute
	 * Il y a aussi la fonction flex_load_velocity() pour fixer la vitesse maximale en pas/s
	*/
	if( (CodeRetourFlex=flex_load_rpm(IdCarte,NIMC_AXIS1,VitesseMaxiAxe1,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpm(): Impossible de configurer la vitesse maximale pour l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpm(IdCarte,NIMC_AXIS2,VitesseMaxiAxe2,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpm(): Impossible de configurer la vitesse maximale pour l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpm(IdCarte,NIMC_AXIS3,VitesseMaxiAxe3,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpm(): Impossible de configurer la vitesse maximale pour l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpm(IdCarte,NIMC_AXIS4,VitesseMaxiAxe4,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpm(): Impossible de configurer la vitesse maximale pour l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration de l'acceleration/decceleration maximale de chaque axe en tours/s par seconde
	*/
	if( (CodeRetourFlex=flex_load_rpsps(IdCarte,NIMC_AXIS1,NIMC_BOTH,AccelDecelMaxiAxe1,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpsps(): Impossible de configurer l'acceleration/deceleration maximale pour l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpsps(IdCarte,NIMC_AXIS2,NIMC_BOTH,AccelDecelMaxiAxe2,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpsps(): Impossible de configurer l'acceleration/deceleration maximale pour l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpsps(IdCarte,NIMC_AXIS3,NIMC_BOTH,AccelDecelMaxiAxe3,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpsps(): Impossible de configurer l'acceleration/deceleration maximale pour l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_load_rpsps(IdCarte,NIMC_AXIS4,NIMC_BOTH,AccelDecelMaxiAxe4,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_load_rpsps(): Impossible de configurer l'acceleration/deceleration maximale pour l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Initialisation de la position des codeurs a la valeur nulle
	*/
	if( (CodeRetourFlex=flex_reset_pos(IdCarte,NIMC_AXIS1,0,0,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_reset_pos(): Impossible d'initialiser la position du codeur 1 a la valeur nulle: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_reset_pos(IdCarte,NIMC_AXIS2,0,0,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_reset_pos(): Impossible d'initialiser la position du codeur 2 a la valeur nulle: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_reset_pos(IdCarte,NIMC_AXIS3,0,0,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_reset_pos(): Impossible d'initialiser la position du codeur 3 a la valeur nulle: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_reset_pos(IdCarte,NIMC_AXIS4,0,0,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_reset_pos(): Impossible d'initialiser la position du codeur 4 a la valeur nulle: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	
	/* Type de mouvement
	*/
	if( (CodeRetourFlex=flex_set_op_mode(IdCarte,NIMC_AXIS1,NIMC_ABSOLUTE_POSITION)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_op_mode(): Impossible de configurer le type de mode de fonctionnement pour l'axe 1: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	if( (CodeRetourFlex=flex_set_op_mode(IdCarte,NIMC_AXIS2,/* NIMC_VELOCITY */ NIMC_ABSOLUTE_POSITION)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_op_mode(): Impossible de configurer le type de mode de fonctionnement pour l'axe 2: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_set_op_mode(IdCarte,NIMC_AXIS3,/* NIMC_VELOCITY */ NIMC_ABSOLUTE_POSITION)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_op_mode(): Impossible de configurer le type de mode de fonctionnement pour l'axe 3: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	if( (CodeRetourFlex=flex_set_op_mode(IdCarte,NIMC_AXIS4,/* NIMC_VELOCITY */ NIMC_ABSOLUTE_POSITION)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_op_mode(): Impossible de configurer le type de mode de fonctionnement pour l'axe 4: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* On desactive les entrees des switchs des positions de repos
	*/
	if( (CodeRetourFlex=flex_enable_home_inputs(IdCarte,0x00)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_enable_home_inputs(): Impossible de desactiver les entrees des switchs des positions de repos des axes: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	
	/* On active les entrees des switchs des limites Reverse et Forward
	*/
	if( (CodeRetourFlex=flex_enable_limits(IdCarte,NIMC_LIMIT_INPUTS,0x1E,0x1E)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_enable_limits(): Impossible d'activer les entrees des switchs des limites des axes: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Configuration de la polarite des switchs de limite Forward et Reverse
	 *
	 * inversee     = active ou niveau bas, mise a la masse (toutes inversees = 0x1E)
	 * non-inversee = active au niveau haut
	*/
	if( (CodeRetourFlex=flex_set_limit_polarity(IdCarte,0x1E,0x1E)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_limit_polarity(): Impossible de configurer la polarite des switchs limites Forward et Reverse: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}
	
	
	/* Configuration de la polarite des switchs de positions de repos
	 *
	 * inversee     = active ou niveau bas, mise a la masse (toutes inversees = 0x1E)
	 * non-inversee = active au niveau haut
	*/
	if( (CodeRetourFlex=flex_set_home_polarity(IdCarte,0x1E)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_set_home_polarity(): Impossible de configurer la polarite des switchs de positions de repos: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	
	/* Si il y en a, affichage et purge des erreurs modales
	*/
	while( (RetourTestErreurModale=TestRecupCompChaineErreurFlexMotionModale(&IdCommandeErr,&IdRessourceErr,&CodeErreurModaleErr,ChaineErreurModale)) )
	{
		ErreurModale|=RetourTestErreurModale;
		BIO_printf(comm,"ROBOA: ERREUR MODALE: %s.\n",ChaineErreurModale);
	}
	
	if( ErreurModale ) return FALSE; else return TRUE;
}


/* FONCTION D'ARRET DOUX ET DE DESACTIVATION DE TOUS LES AXES
 *
 * CE:	On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *
 * CS:	La fonction est vraie dans le cas d'une recherche positive, fausse dans l'autre cas.
*/

int ArretDouxDesactivationAxes(BIO *comm)
{
	u16 ResultatMouv;		/* Resultat de l'arret du mouvement                                                    */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale        */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */
	
	/* Arret du mouvement de TOUS les axes
	*/
	if( (CodeRetourFlex=flex_stop_motion(IdCarte,0,NIMC_DECEL_STOP,0x1E)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement des axes: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* On attend que tous les axes soient arretes avant de continuer (attente de la deceleration) TimeOut de 60s, test tous les 1/10 s
	*/
	if( (CodeRetourFlex=flex_wait_for_move_complete(IdCarte,0,0x1E,60000,100,&ResultatMouv)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_wait_for_move_complete_status(): Impossible d'attendre l'arret des axes: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Desactivation de tous les axes
	*/
	if( (CodeRetourFlex=flex_enable_axes(IdCarte,NIMC_AXIS_CTRL,0,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_enable_axes(): Impossible de desactiver des axes: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}

	return TRUE;
}


/* FONCTION DE RECHERCHE DE LA POSITION INDEX (TOP 0) D'UN CODEUR SUR L'AXE n
 *
 * Cette fonction doit etre logiquement appelee apres la recherche de la position "home"
 *  si l'axe en dispose d'une.
 * 
 * CE:	On passe le numero de l'axe ;
 *
 *      On passe le temps impartie pour la recherche de l'index de l'axe en millisecondes ;
 *
 *      On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *
 * CS:	La fonction est vraie dans le cas d'une recherche positive, fausse dans l'autre cas
*/

int RechercherIndexCodeur(u8 axe,u32 timeout,BIO *comm)
{
	u16 BitmapStatusAxe;		/* Bitmap du status renseigne d'un axe                                                 */
	u16 ResultatMouv;		/* Resultat de l'arret du mouvement                                                    */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale        */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */


	/* Cette fonctionnalite doit etre executee uniquement sur un axe arrete ou tue.
	 * On commence donc par arreter l'axe en question
	*/
	if( (CodeRetourFlex=flex_stop_motion(IdCarte,axe,NIMC_DECEL_STOP,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	/* On attend que l'axe soit arrete avant de continuer (attente de la deceleration), test tous les 1/10 s
	*/
	if( (CodeRetourFlex=flex_wait_for_move_complete(IdCarte,axe,0,timeout,100,&ResultatMouv)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_wait_for_move_complete(): Impossible d'attendre l'arret de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	/* Si l'axe est bien en arret
	*/
	if( ResultatMouv )
	{
		time_t TempsDebut;		/* Le temps pour gerer le time out de recherche de l'index */
		time_t TempsCourant;		/* Le temps pour gerer le time out de recherche de l'index */

		/* On sauvegarde le temps juste avant le declanchement de la recherche de l'index
		*/
		time(&TempsDebut);
		
		/* On declanche la recherche de l'index du codeur de l'axe
		 *
		 * ATTENTION : Pour que l'origine soit trouvee il faut absolument que la polarite des phases des codeurs soient bien respectee pour
		 *  la carte controleur NI. Le top 0 doit avoir lieu avec la phase A et la phase B a une valeur nulle : A=B=0 Index=1. Il faut donc
		 *  brancher le codeur pour respecter cette polarite. On peut regler ce phasage en inverssant A et A barre, et, B et B barre.
		 *  Il faut aussi respecter le sens de rotation du moteur en lien avec le codeur. Logiquement NIMC_FORWARD_DIRECTION doit faire
		 *  tourner le moteur dans le sens des aiguilles d'une montre lorsqu'on le regarde de face. Le codeur doit alors incrementer le
		 *  comptage. Si ces reglages ne sont pas respectes, la fonction flex_find_index() a un comportement foireux et l'index n'est pas
		 *  retrouve. On peut changer le sens de comptage du codeur en inversant les phases A et B.
		*/
		if( (CodeRetourFlex=flex_find_index(IdCarte,axe,NIMC_FORWARD_DIRECTION,0)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_find_index(): Impossible de rechercher l'index du codeur de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
			return FALSE;
		}

		/* On test si on a bien trouve la position index du codeur de l'axe
		*/
		do
		{
			/* Le programme patiente x microsecondes entre chaque test
			*/
			usleep(100000);

			/* On sauvegarde le temps courant
			*/
			time(&TempsCourant);

			/* Si on a depasse le temps impartie pour la recherche de l'index du codeur
			*/
			if( difftime(TempsCourant,TempsDebut) > ((double) TIMEOUT_RECH_INDEX_CODEUR)/1000.0 )
			{
				/* Arret immediat du mouvement de l'axe
				*/
				if( (CodeRetourFlex=flex_stop_motion(IdCarte,axe,NIMC_KILL_STOP,0)) != NIMC_noError )
				{
					CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
					BIO_printf(comm,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement de l'axe %d dans le cas d'une recherche non trouvee de l'index du codeur: %s.\n",axe,ChaineErreurNonModale);
					return FALSE;
				}

				return FALSE;
			}

			/* On test le bit "index trouve"
			*/
			if( (CodeRetourFlex=flex_read_axis_status_rtn(IdCarte,axe,&BitmapStatusAxe)) != NIMC_noError )
			{
				CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
				BIO_printf(comm,"ROBOA: ERREUR: flex_read_axis_status_rtn(): Impossible de lire le bitmap du status des axes: %s.\n",ChaineErreurNonModale);
				return FALSE;
			}
			
		} while( !(BitmapStatusAxe & NIMC_INDEX_FOUND_BIT) );
		
		/* Initialisation de la position du codeur a la valeur nulle
		*/
		if( (CodeRetourFlex=flex_reset_pos(IdCarte,axe,0,0,0xFF)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_reset_pos(): Impossible d'initialiser la position du codeur %d a la valeur nulle dans la recherche de son index: %s.\n",axe,ChaineErreurNonModale);
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}

	if( BitmapStatusAxe & NIMC_INDEX_FOUND_BIT ) return TRUE; else return FALSE;
}


/* FONCTION DE RECHERCHE DE LA POSITION DE REPOS DE L'AXE n
 *
 * Cette fonction doit etre logiquement appelee dans le cas d'une boucle de controle ouverte
 *  ou avant la recherche de l'index codeur dans le cas d'une boucle de controle fermee mais
 *  de n'est pas obligatoire dans le cas d'une boucle de controle fermee.
 * 
 * CE:	On passe le numero de l'axe ;
 *
 *      On passe le temps impartie pour la recherche de la position de repos de l'axe en millisecondes ;
 *
 *      On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *
 * CS:	La fonction est vraie dans le cas d'une recherche positive, fausse dans l'autre cas
*/

int RechercherPositionRepos(u8 axe,u32 timeout,BIO *comm)
{
	u16 BitmapStatusAxe;		/* Bitmap du status renseigne d'un axe                                                 */
	u16 ResultatMouv;		/* Resultat de l'arret du mouvement                                                    */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale        */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */


	/* Cette fonctionnalite doit etre executee uniquement sur un axe arrete ou tue.
	 * On commence donc par arreter l'axe en question
	*/
	if( (CodeRetourFlex=flex_stop_motion(IdCarte,axe,NIMC_DECEL_STOP,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	/* On attend que l'axe soit arrete avant de continuer (attente de la deceleration), test tous les 1/10 s
	*/
	if( (CodeRetourFlex=flex_wait_for_move_complete(IdCarte,axe,0,timeout,100,&ResultatMouv)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_wait_for_move_complete(): Impossible d'attendre l'arret de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	/* Si l'axe est bien en arret
	*/
	if( ResultatMouv )
	{
		int bitaxe=0;			/* Bit correspondant a l'entree de la position de repos de l'axe        */
		time_t TempsDebut;		/* Le temps pour gerer le time out de recherche de la position de repos */
		time_t TempsCourant;		/* Le temps pour gerer le time out de recherche de la position de repos */

		/* On sauvegarde le temps juste avant le declanchement de la recherche de la position de repos
		*/
		time(&TempsDebut);
		
		/* On active l'entree du switch de la position de repos de l'axe
		*/
		switch( axe )
		{
			case NIMC_AXIS1:		bitaxe=2;	break;
			case NIMC_AXIS2:		bitaxe=4;	break;
			case NIMC_AXIS3:		bitaxe=8;	break;
			case NIMC_AXIS4:		bitaxe=16;	break;
			default:			bitaxe=0x1E;	break;
		}

		if( (CodeRetourFlex=flex_enable_home_inputs(IdCarte,bitaxe)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_enable_home_inputs(): Impossible de desactiver les entrees des switchs des positions de repos des axes: %s.\n",ChaineErreurNonModale);
			return FALSE;
		}

		/* On declanche la recherche de la position de repos de l'axe
		 *
		 * ATTENTION : Pour que la position de repos soit trouvee, il faut respecter le sens de rotation du moteur. La direction Forward doit
		 *  faire tourner le moteur dans le sens des aiguilles d'une montre lorsqu'on le regarde de face.
		 *
		 * +1 approche finale du switch dans le sens reverse
		 * +2 direction de recherche initiale dans le sens reverse
		 * +4 front d'arret cote reverse (le switch produit deux fronts : un front cote reverse, l'autre cote forward)
		 *
		 * 1 : On part a la recherche dans le sens forward, on s'arretera dans le sens reverse au niveau du front cote forward
		*/
		if( (CodeRetourFlex=flex_find_home(IdCarte,axe,1)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_find_home(): Impossible de rechercher la position de repos de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
			return FALSE;
		}

		/* On test si on a bien trouve la position de repos de l'axe
		 *
		 * ATTENTION : SI LES DEUX LIMITES SONT DECLANCHEES DURANT LES MOUVEMENTS DE LA RECHERCHE ET QUE LA POSITION DE REPOS N'A PAS ETE TROUVEE
		 *  ALORS IL FAUT ATTENDRE LE TIMEOUT POUR SORTIR DE LA FONCTION. LE FONCTION SERA FAUSSE AVEC UNE ERREUR MODALE.
		*/
		do
		{
			/* Le programme patiente x microsecondes entre chaque test
			*/
			usleep(100000);

			/* On sauvegarde le temps courant
			*/
			time(&TempsCourant);

			/* Si on a depasse le temps impartie pour la recherche de la position de repos
			*/
			if( difftime(TempsCourant,TempsDebut) > ((double) TIMEOUT_RECH_POSITION_REPOS)/1000.0 )
			{
				/* Arret immediat du mouvement de l'axe
				*/
				if( (CodeRetourFlex=flex_stop_motion(IdCarte,axe,NIMC_KILL_STOP,0)) != NIMC_noError )
				{
					CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
					BIO_printf(comm,"ROBOA: ERREUR: flex_stop_motion(): Impossible de stopper le mouvement de l'axe %d dans le cas d'une recherche de la position de repos: %s.\n",axe,ChaineErreurNonModale);
					return FALSE;
				}

				return FALSE;
			}

			/* On test le bit "position de repos trouvee"
			*/
			if( (CodeRetourFlex=flex_read_axis_status_rtn(IdCarte,axe,&BitmapStatusAxe)) != NIMC_noError )
			{
				CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
				BIO_printf(comm,"ROBOA: ERREUR: flex_read_axis_status_rtn(): Impossible de lire le bitmap du status des axes: %s.\n",ChaineErreurNonModale);
				return FALSE;
			}

		} while( !(BitmapStatusAxe & NIMC_HOME_FOUND_BIT) );

		
		/* On desactive les entrees des switchs des positions de repos car les axes sont stoppes par le passage sur les switchs
		*/
		if( (CodeRetourFlex=flex_enable_home_inputs(IdCarte,0x00)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			BIO_printf(comm,"ROBOA: ERREUR: flex_enable_home_inputs(): Impossible de desactiver les entrees des switchs des positions de repos des axes: %s.\n",ChaineErreurNonModale);
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}

	if( BitmapStatusAxe & NIMC_HOME_FOUND_BIT ) return TRUE; else return FALSE;
}


/* FONCTION DE LECTURE DE LA POSITION DES AXES EN PAS CODEUR EN QUADRATURE
 *
 * CE:	On passe un pointeur sur i32 qui contiendra la position de l'axe 1 ;
 *
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 2 ;
 *
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 3 ;
 * 	
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 4 ;
 *
 *      On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *      
 * CS:	La fonction est fausse si il y a eu une erreur ;
*/

int PositionsAxesEnPasCodeurQuadrature(i32 *PositionCodeur1,i32 *PositionCodeur2,i32 *PositionCodeur3,i32 *PositionCodeur4,BIO *comm)
{
	int retour=0;		/* Valeur de retour en fonction des erreurs survenues                           */
	i32 CodeRetourFlex;	/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale   */
	
	/* Lecture de la position en pas codeur en quadrature de l'axe 1
	*/
	if( (CodeRetourFlex=flex_read_encoder_rtn(IdCarte,NIMC_AXIS1,PositionCodeur1)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_encoder_rtn(): Impossible de lire la position du codeur 1 en pas codeur: %s.\n",ChaineErreurNonModale);
		retour|=TRUE;
	}

	/* Lecture de la position en pas codeur en quadrature de l'axe 2
	*/
	if( (CodeRetourFlex=flex_read_encoder_rtn(IdCarte,NIMC_AXIS2,PositionCodeur2)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_encoder_rtn(): Impossible de lire la position du codeur 2 en pas codeur: %s.\n",ChaineErreurNonModale);
		retour|=TRUE;
	}

	/* Lecture de la position en pas codeur en quadrature de l'axe 3
	*/
	if( (CodeRetourFlex=flex_read_encoder_rtn(IdCarte,NIMC_AXIS3,PositionCodeur3)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_encoder_rtn(): Impossible de lire la position du codeur 3 en pas codeur: %s.\n",ChaineErreurNonModale);
		retour|=TRUE;
	}

	/* Lecture de la position en pas codeur en quadrature de l'axe 4
	*/
	if( (CodeRetourFlex=flex_read_encoder_rtn(IdCarte,NIMC_AXIS4,PositionCodeur4)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_encoder_rtn(): Impossible de lire la position du codeur 4 en pas codeur: %s.\n",ChaineErreurNonModale);
		retour|=TRUE;
	}

	return ~retour;
}


/* FONCTION DE LECTURE DE LA POSITION DES AXES EN PAS MOTEUR
 *
 * CE:	On passe un pointeur sur i32 qui contiendra la position de l'axe 1 ;
 *
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 2 ;
 *
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 3 ;
 * 	
 * 	On passe un pointeur sur i32 qui contiendra la position de l'axe 4 ;
 *
 *      On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *      
 * CS:	La fonction est fausse si il y a eu une erreur ;
*/

int PositionsAxesEnPasMoteur(i32 *PositionCodeur1,i32 *PositionCodeur2,i32 *PositionCodeur3,i32 *PositionCodeur4,BIO *comm)
{
	int retour=0;		/* Valeur de retour en fonction des erreurs survenues                           */
	i32 CodeRetourFlex;	/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale   */

	/* Lecture de la position en pas moteur de l'axe 1
	*/
	if( (CodeRetourFlex=flex_read_pos_rtn(IdCarte,NIMC_AXIS1,PositionCodeur1)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_pos_rtn(): Impossible de lire la position du codeur 1 en pas moteur: %s.\n",ChaineErreurNonModale);
	}

	/* Lecture de la position en pas moteur de l'axe 2
	*/
	if( (CodeRetourFlex=flex_read_pos_rtn(IdCarte,NIMC_AXIS2,PositionCodeur2)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_pos_rtn(): Impossible de lire la position du codeur 2 en pas moteur: %s.\n",ChaineErreurNonModale);
	}

	/* Lecture de la position en pas moteur de l'axe 3
	*/
	if( (CodeRetourFlex=flex_read_pos_rtn(IdCarte,NIMC_AXIS3,PositionCodeur3)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_pos_rtn(): Impossible de lire la position du codeur 3 en pas moteur: %s.\n",ChaineErreurNonModale);
	}

	/* Lecture de la position en pas moteur de l'axe 4
	*/
	if( (CodeRetourFlex=flex_read_pos_rtn(IdCarte,NIMC_AXIS4,PositionCodeur4)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		BIO_printf(comm,"ROBOA: ERREUR: flex_read_pos_rtn(): Impossible de lire la position du codeur 4 en pas moteur: %s.\n",ChaineErreurNonModale);
	}

	return ~retour;
}


/* FONCTION DE POSITIONNEMENT D'UN AXE
 *
 * CE:	On passe le numero de l'axe a positionner en boucle fermee ;
 *
 * 	On passe la position de destination signee en pas moteur (ou micropas) puisque il s'agit d'une bouble fermee avec des moteurs pas a pas ;
 * 	
 * CS:	La fonction est vraie si aucune erreur non modale n'est signalee ;
*/

int PositionnerAxe(u8 axe,i32 destination)
{
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreurs non-modale        */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */

	/* On charge la position de destination
	*/
	if( (CodeRetourFlex=flex_load_target_pos(IdCarte,axe,destination,0xFF)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		fprintf(stderr,"ROBOA: ERREUR: flex_load_target_pos(): Impossible de charger la position de destination de l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	/* On demarre le mouvement du moteur de l'axe immediatement sans attendre la fin d'un autre
	*/
	if( (CodeRetourFlex=flex_start(IdCarte,axe,0)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		fprintf(stderr,"ROBOA: ERREUR: flex_start(): Impossible de positionner l'axe %d: %s.\n",axe,ChaineErreurNonModale);
		return FALSE;
	}

	return TRUE;
}


/* FONCTION DE COMPOSITION DE LA CHAINE D'UNE ERREUR NON-MODALE
 *
 * Une erreur non modale est detectee a l'appel d'une fonction FlexMotion.
 *
 * CE:	On passe le code d'erreur (status de retour de la fonction FlexMotion) ;
 *
 * 	On passe un pointeur sur char qui contiendra la chaine de description de l'erreur ;
 *
 * CS:	La fonction ne retourne rien ;
*/

void CompChaineErreurFlexMotionNonModale(i32 CodeErreur,char *ChaineErreur)
{
	u32 NbCaracteres=0;	/* Nombre de caracteres de la chaine d'erreur                                          */
	i32 CodeRetourFlex;	/* Code de retour des fonctions de la librairie FlexMotion = Erreur non modale         */


	/* Recuperation du nombre de caracteres de la description de l'erreur non modale
	*/
	if( (CodeRetourFlex=flex_get_error_description(NIMC_ERROR_ONLY,CodeErreur,0,0,NULL,&NbCaracteres)) != NIMC_noError )
	{
		fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): impossible de recuperer le nombre de caracteres de la chaine de la description de l'erreur non modale.\n");
		return;
	}

	/* Si la chaine n'est pas plus longue que notre tableau
	*/
	if( NbCaracteres < TAILLE_MAXI_CHAINE_ERREUR-1 )
	{
		/* Composition et recuperation de la chaine de description de l'erreur non modale
		*/
		if( (CodeRetourFlex=flex_get_error_description(NIMC_ERROR_ONLY,CodeErreur,0,0,ChaineErreur,&NbCaracteres)) != NIMC_noError )
		{
			fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): impossible de recuperer la chaine de la description de l'erreur non modale.\n");
		}
	}
	else
	{
		int i;					/* Indice                                                                          */
		char ChaineComplete[NbCaracteres+1];	/* Tableau pour pouvoir recuperer la chaine complete de la description de l'erreur */

		/* Composition et recuperation de la chaine de description de l'erreur non modale
		*/
		if( (CodeRetourFlex=flex_get_error_description(NIMC_ERROR_ONLY,CodeErreur,0,0,ChaineErreur,&NbCaracteres)) != NIMC_noError )
		{
			fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): impossible de recuperer la chaine de la description de l'erreur non modale.\n");
		}
		
		/* On recupere ce que l'on peut de la chaine
		*/
		for( i=0; i < TAILLE_MAXI_CHAINE_ERREUR-1; i++ ) *(ChaineErreur+i)=ChaineComplete[i];
		*(ChaineErreur+TAILLE_MAXI_CHAINE_ERREUR-1)=0;
	}
}


/* FONCTION DE TEST, DE RECUPERATION ET DE COMPOSITION DE LA CHAINE DE LA DERNIERE ERREUR MODALE FLEXMOTION
 *
 * Une erreur modale est une erreur qui n'est pas detectee durant l'appel d'une fonction mais plutot durant
 *  l'execution. Elles sont stockees dans une FIFO du controleur car toutes les erreurs ne sont pas
 *  detectables durant l'execution des fonctions. Elles sont decrites par trois donn�es : les identifieurs
 *  de commande, ressource et le code d'erreur. Le bit ErrMsg du CSR est aussi positionne lorsqu'une erreur
 *  est dans la FIFO.
 *  
 * CE:	On passe un pointeur sur u16 qui contiendra l'identifieur de la commande (ce qu'on a demande) ;
 *
 *	On passe un pointeur sur u16 qui contiendra l'identifieur de la ressource (l'axe...) ;
 *
 *	On passe un pointeur sur i32 qui contiendra le code d'erreur (numero de l'erreur) ;
 *
 *	On passe un pointeur sur char qui contiendra la chaine composee de l'erreur ;
 *
 * CS:	La fonction est vraie si une erreur modale est trouvee dans la FIFO du controleur ;
*/

int TestRecupCompChaineErreurFlexMotionModale(u16 *IdCommande,u16 *IdRessource,i32 *CodeErreurModale,char *ChaineErreur)
{
	u16 CSR;			/* Registre Communications Status Register                                             */
	i32 CodeRetourFlex;		/* Code de retour des fonctions de la librairie FlexMotion = Erreur non modale         */
	char ChaineErreurNonModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur non modale                  */


	/* Lecture du CSR pour savoir si une erreur modale est presente dans la FIFO
	*/
	if( (CodeRetourFlex=flex_read_csr_rtn(IdCarte,&CSR)) != NIMC_noError )
	{
		CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
		fprintf(stderr,"ROBOA: ERREUR: flex_read_csr_rtn(): Impossible de lire le registre CSR: %s.\n",ChaineErreurNonModale);
		return FALSE;
	}


	/* Si il y a bien une erreur modale dans le controleur
	*/
	if( CSR & NIMC_MODAL_ERROR_MSG )
	{
		u32 NbCaracteres;	/* Nombre de caracteres de la chaine de description de l'erreur */
		
		/* Lecture de l'erreur modale la plus recente de la pile des messages d'erreur
		*/
		if( (CodeRetourFlex=flex_read_error_msg_rtn(IdCarte,IdCommande,IdRessource,CodeErreurModale)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			fprintf(stderr,"ROBOA: ERREUR: flex_read_error_msg_rtn(): Impossible de lire la derniere erreur modale de la FIFO des erreurs: %s.\n",ChaineErreurNonModale);
			return TRUE;
		}

		/* Recuperation de la description combinee et complete de l'erreur modale
		*/
		NbCaracteres=0;		/* Condition d'entree pour recuperer le nombre de caracteres */
		
		if( (CodeRetourFlex=flex_get_error_description(NIMC_COMBINED_DESCRIPTION,*CodeErreurModale,*IdCommande,*IdRessource,NULL,&NbCaracteres)) != NIMC_noError )
		{
			CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
			fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): Impossible de recuperer la dimension de la chaine du message d'erreur compose pour la derniere erreur modale: %s.\n",ChaineErreurNonModale);
			return TRUE;
		}

		/* Si la chaine n'est pas trop longue pour notre tableau
		*/
		if( NbCaracteres < TAILLE_MAXI_CHAINE_ERREUR-1 )
		{
			if( (CodeRetourFlex=flex_get_error_description(NIMC_COMBINED_DESCRIPTION,*CodeErreurModale,*IdCommande,*IdRessource,ChaineErreur,&NbCaracteres)) != NIMC_noError )
			{
				CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
				fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): Impossible de recuperer le message d'erreur combine pour la derniere erreur modale: %s.\n",ChaineErreurNonModale);
				return TRUE;
			}
		}
		else
		{
			int i;					/* Indice                                                                          */
			char ChaineComplete[NbCaracteres+1];	/* Tableau pour pouvoir recuperer la chaine complete de la description de l'erreur */

			if( (CodeRetourFlex=flex_get_error_description(NIMC_COMBINED_DESCRIPTION,*CodeErreurModale,*IdCommande,*IdRessource,ChaineErreur,&NbCaracteres)) != NIMC_noError )
			{
				CompChaineErreurFlexMotionNonModale(CodeRetourFlex,ChaineErreurNonModale);
				fprintf(stderr,"ROBOA: ERREUR: flex_get_error_description(): Impossible de recuperer le message d'erreur combine pour la derniere erreur modale: %s.\n",ChaineErreurNonModale);
				return TRUE;
			}
			
			/* On recupere ce que l'on peut de la chaine
			*/
			for( i=0; i < TAILLE_MAXI_CHAINE_ERREUR-1; i++ ) *(ChaineErreur+i)=ChaineComplete[i];
			*(ChaineErreur+TAILLE_MAXI_CHAINE_ERREUR-1)=0;
		}

		return TRUE;
	}

	return FALSE;
}


/* FONCTION D'AFFICHAGE ET DE PURGE DES ERREURS MODALES
 *
 * CE:	On passe un pointeur sur l'objet BIO de la communication client - serveur ROBOA ;
 *
 * CS:	-
*/

void AffichagePurgeErreursModales(BIO *comm)
{
	u16 IdCommandeErr;					/* Numero de commande d'une erreur modale                        */
	u16 IdRessourceErr;					/* Numero de ressource d'une erreur modale                       */
	i32 CodeErreurModaleErr;				/* Code d'erreur d'une erreur modale                             */
	char ChaineErreurModale[TAILLE_MAXI_CHAINE_ERREUR];	/* Chaine description d'une erreur modale                        */
	
	/* Affichage et purge des erreurs modales
	*/
	while( TestRecupCompChaineErreurFlexMotionModale(&IdCommandeErr,&IdRessourceErr,&CodeErreurModaleErr,ChaineErreurModale) )
	{
		char ChainePuts[TAILLE_MAXI_CHAINE_BIO];	/* Chaine de composition */
		
		sprintf(ChainePuts,"ROBOA: ERREUR MODALE: %s.\n",ChaineErreurModale);

		fprintf(stderr,ChainePuts);

		BIO_printf(comm,ChainePuts);
	}
}


/* FONCTION D'AFFICHAGE DES ERREURS PRODUITES SUR LA PILE DE LA LIBRAIRIE SSL
 *
 * CE:	-
 *
 * CS:	-
*/

void AfficheErreurPileLibrairieSSL(void)
{
	unsigned long numero;				/* Numero de l'erreur de la librairie SSL */
	char Chaine[TAILLE_MAXI_CHAINE_ERREUR_SSL];	/* Chaine de composition des erreurs      */
	
	while( (numero=ERR_get_error()) != 0 )
	{
		ERR_error_string_n(numero,Chaine,TAILLE_MAXI_CHAINE_ERREUR_SSL);

		fprintf(stderr,"ROBOA: ERREUR: SSL : %s.\n",Chaine);
	}
}


/* FONCTION D'AFFICHAGE DES ERREURS PRODUITES PAR LES OPERATIONS E/S TLS/SSL
 *
 * CE:	On passe un pointeur sur la structure de la connexion SSL ;
 *
 *	On passe la valeur retournee par la fonction d'e/s TLS/SSL ;
 *
 * CS:	-
*/

void AfficheErreurES_SSL(SSL *structure,int retour)
{
	switch( SSL_get_error(structure,retour) )
	{
		case SSL_ERROR_NONE:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_NONE: Aucune erreur d'e/s.\n");
			break;
		case SSL_ERROR_ZERO_RETURN:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_ZERO_RETURN: La connexion TLS/SSL a ete fermee par une alerte du protocole.\n");
			break;
		case SSL_ERROR_WANT_READ:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_WANT_READ: L'operation de lecture n'a pu se realiser.\n");
			break;
		case SSL_ERROR_WANT_WRITE:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_WANT_WRITE: L'operation d'ecriture n'a pu se realiser.\n");
			break;
		case SSL_ERROR_WANT_CONNECT:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_WANT_CONNECT: L'operation de connexion n'a pu se realiser.\n");
			break;
/*		case SSL_ERROR_WANT_ACCEPT:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_WANT_ACCEPT: L'operation de negociation SSL n'a pu se realiser.\n");
			break;
*/
		case SSL_ERROR_WANT_X509_LOOKUP:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_WANT_X509_LOOKUP: L'operation n'a pu se realiser car la fonction de retour SSL_CTX_set_client_cert_cb() doit etre appelee une nouvelle fois.\n");
			break;
		case SSL_ERROR_SYSCALL:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_SYSCALL: Erreurs d'appel systeme d'entrees/sorties .\n");
			break;
		case SSL_ERROR_SSL:
			fprintf(stderr,"ROBOA: ERREUR SSL_ERROR_SSL: Erreur dans la librairie SSL, sans doute une erreur de protocole.\n");
			break;
	}
}


/* FONCTION APPELEE PAR LA LIBRAIRIE SSL POUR LIRE OU STOCKER LES FICHIERS PEM CONTENANT UNE CLE CHIFFREE
 *
 * CE:	La librairie SSL passe un pointeur vers le tableau ou doit etre copie le mot de passe ;
 * 
 * 	La librairie SSL passe la dimension maximale du mot de passe ;
 *
 * 	La librairie SSL passe 0 si la fonction est utilisee pour lire/decryptee, ou, 1 si la fonction est appelee pour ecrire/encryptee
 *
 * 	La librairie SSL passe un pointeur vers une donnee passee par la routine PEM. Il permet qu'une donnee arbitraire soit passee
 * 	 a cette fonction par une application (comme par exemple un identifieur de fenetre dans une application graphique).
 *
 * CS:	La fonction doit retourner la longueur du mot de passe.
*/

int FnMotDePasseCleChiffree(char *buf, int size, int rwflag, void *data)
{
	if( size < strlen(MotDePasseClePriveeServeurROBOA)+1 ) return 0;

	strcpy(buf,MotDePasseClePriveeServeurROBOA);

	return( strlen(buf) );
}


/* FONCTION DE HANDLER DU SIGNAL SIGPIPE
 *
 * CE:	Le systeme passe le signal ;
 *
 * CS:	-
*/

void FnHandlerSIGPIPE(int signal)
{
	fprintf(stderr,"ROBOA: Signal ->SIGPIPE<- recu par le processus.\n");
}

