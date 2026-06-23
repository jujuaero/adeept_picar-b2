import cv2
import os

# 1. Chargement direct des images de reference
path = 'images'
images = []
classNames = []

if not os.path.exists(path):
    print(f"Erreur : Le dossier '{path}' n'existe pas.")
    exit()

myList = os.listdir(path)
print("Chargement des panneaux de reference...")

for cl in myList:
    # Lecture directe en niveaux de gris (0)
    imgCur = cv2.imread(f'{path}/{cl}', 0) 
    if imgCur is not None:
        images.append(imgCur)
        classNames.append(os.path.splitext(cl)[0])

print(f"Panneaux prets : {classNames}")

# 2. Initialisation ORB
orb = cv2.ORB_create(nfeatures=1000)

def findDescriptors(images):
    desList = []
    for img in images:
        kp, des = orb.detectAndCompute(img, None)
        if des is not None:
            desList.append(des)
        else:
            print("Erreur : Impossible d'extraire des points d'interet.")
    return desList

desList = findDescriptors(images)
print("Empreintes calculees. Demarrage de la camera...")

# 3. Fonction de comparaison
def findID(img, desList, thres=15):
    kp2, des2 = orb.detectAndCompute(img, None)
    if des2 is None:
        return -1
        
    bf = cv2.BFMatcher()
    matchList = []
    finalVal = -1
    
    try:
        for des in desList:
            matches = bf.knnMatch(des, des2, k=2)
            good = []
            for m, n in matches:
                # Test de ratio pour eliminer les faux positifs
                if m.distance < 0.75 * n.distance:
                    good.append([m])
            matchList.append(len(good))
    except Exception as e:
        pass
    
    if len(matchList) != 0:
        if max(matchList) > thres:
            finalVal = matchList.index(max(matchList))
    return finalVal

# 4. Boucle video principale
vidcap = cv2.VideoCapture(0, cv2.CAP_V4L2)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

if not vidcap.isOpened():
    print("Erreur : Impossible d'ouvrir la camera")
    exit()

while True:
    ret, frame = vidcap.read()
    if not ret:
        continue
        
    # Conversion du flux en niveaux de gris
    imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    id_panneau = findID(imgGray, desList, thres=15)
    
    if id_panneau != -1:
        nom = classNames[id_panneau]
        cv2.putText(frame, f"Panneau: {nom}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        print(f"Detecte : {nom}")

    cv2.imshow('Camera - Reconnaissance', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vidcap.release()
cv2.destroyAllWindows()