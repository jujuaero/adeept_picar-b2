from spi_ws2812 import Adeept_SPI_LedPixel

# 14 LED WS2812 : 2 integrees au HAT + 4 modules de 3 LED
led = Adeept_SPI_LedPixel(14, 50, 'GRB')

def set_led(numero_led, couleur, intensite=255):
    """
    numero_led : de 0 a 13
    couleur : 'R', 'G', 'B' ou 'N'
    intensite : de 0 a 255
    """

    if numero_led < 0 or numero_led > 13:
        print("Erreur : numero LED invalide")
        return

    if intensite < 0:
        intensite = 0
    if intensite > 255:
        intensite = 255

    couleur = couleur.upper()

    if couleur == "R":
        led.set_led_color(numero_led, intensite, 0, 0)
    elif couleur == "G":
        led.set_led_color(numero_led, 0, intensite, 0)
    elif couleur == "B":
        led.set_led_color(numero_led, 0, 0, intensite)
    elif couleur == "N":
        led.set_led_color(numero_led, 0, 0, 0)
    else:
        print("Erreur : couleur invalide. Utiliser R, G, B ou N")

def protocole_manuel():
    print("Commande : numero_led couleur intensite")
    print("Exemple : 3 R 255")
    print("Couleurs : R, G, B, N")
    print("N = eteindre")
    print("q = quitter")

    while True:
        commande = input("Commande : ").strip()

        if commande.lower() == "q":
            break

        elements = commande.split()

        if len(elements) == 2:
            numero = int(elements[0])
            couleur = elements[1]
            set_led(numero, couleur)

        elif len(elements) == 3:
            numero = int(elements[0])
            couleur = elements[1]
            intensite = int(elements[2])
            set_led(numero, couleur, intensite)

        else:
            print("Commande invalide")

def tout_eteindre():
    for i in range(14):
        set_led(i, "N")

try:
    tout_eteindre()
    protocole_manuel()

finally:
    tout_eteindre()
    led.led_close()