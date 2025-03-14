def process_image_circle(self, img, depth):
	    """
	    Processing of an image with a circle.
	    :param img: opencv image (in HSV)
	    :return img: processed opencv image
	    """
	    # Convertir l'image en niveaux de gris (en utilisant uniquement la composante de valeur, V)
	    gray = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)  # Convertir en BGR pour un meilleur rendu en niveaux de gris
	    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

	    # Appliquer un flou médian pour réduire le bruit
	    gray = cv2.medianBlur(gray, 5)

	    # Appliquer la transformation de Hough pour détecter les cercles
	    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
		                        param1=50, param2=30, minRadius=0, maxRadius=0)
	    #print(circles)
	    # Vérifier si des cercles sont détectés
	    if circles is not None:
		circles = np.uint16(np.around(circles))  # Convertir les coordonnées des cercles en entiers

		# Dessiner les cercles et leurs centres sur l'image
		for i in circles[0, :]:
		    # Dessiner le cercle extérieur
		    cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
		    # Dessiner le centre du cercle
		    cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

		# Initialiser la fenêtre de suivi à partir du premier cercle détecté
		x, y, radius = i[0], i[1], i[2]
		# Vérification pour éviter les indices négatifs
		x = max(x - radius, 0)
		track_window = (x, y, 2 * radius, 2 * radius)

		# Convertir l'image en HSV pour appliquer MeanShift
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# S'assurer que les indices sont dans les limites de l'image
		roi_x1 = max(x, 0)
		roi_y1 = max(y, 0)
		roi_x2 = min(x + 2 * radius, img.shape[1])  # largeur maximale de l'image
		roi_y2 = min(y + 2 * radius, img.shape[0])  # hauteur maximale de l'image
		roi = hsv[roi_y1:roi_y2, roi_x1:roi_x2]

		roi_hist = cv2.calcHist([roi], [0], None, [180], [0, 180])
		cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

		# Calculer le backprojection pour MeanShift
		dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

		# Appliquer MeanShift pour le suivi du cercle
		ret, track_window = cv2.meanShift(dst, track_window, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
		print("_____________________________________________________")		
		print(track_window)
		# Si le suivi est réussi, dessiner la nouvelle position
		if ret:
		    x, y, w, h = track_window
		    img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
		    cv2.putText(img, "Tracking Active", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		else:
		    # Si le suivi échoue, afficher un message d'erreur
		    cv2.putText(img, "Tracking Failed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

	    else:
		# Si aucun cercle n'est détecté, afficher un message d'erreur
		cv2.putText(img, "No Circle Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

	    # Retourner l'image avec les annotations visuelles
	    return img

