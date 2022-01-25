import random
import matplotlib.pyplot as plt
from detection import *


def longueur (x,y, ordre):
    i = ordre[-1]
    x0,y0 = x[i], y[i]
    d = 0
    for o in ordre:
        x1,y1 = x[o], y[o]
        d += (x0-x1)**2 + (y0-y1)**2
        x0,y0 = x1,y1
    return d

def permutation_rnd(x,y,ordre,miniter):
    d  = longueur(x,y,ordre)
    d0 = d+1
    it = 1
    while d < d0 or it < miniter :
        it += 1
        d0 = d
        for i in range(1,len(ordre)-1) :
            for j in range(i+2,len(ordre)+ 1):
                k = random.randint(1,len(ordre)-1)
                l = random.randint(k+1,len(ordre))
                r = ordre[k:l].copy()
                r.reverse()
                ordre2 = ordre[:k] + r + ordre[l:]
                t = longueur(x,y,ordre2)
                if t < d :
                    d = t
                    ordre = ordre2
    return ordre

def n_permutation(x,y, miniter):
    ordre = list(range(len(x)))
    bordre = ordre.copy()
    d0 = longueur(x,y,ordre)
    for i in range(0,20):
        print("iteration",i, "d=",d0)
        random.shuffle(ordre)
        ordre = permutation_rnd (x,y,ordre, 20)
        d = longueur(x,y,ordre)
        if d < d0 :
            d0 = d
            bordre = ordre.copy()
    return bordre


if __name__ == "__main__":
    image_name = 'balls.png'
    img = cv2.imread(image_name)

    ball_img, balls = detect_balls(img)

    zone_img, zones = detect_zones(img)

    n = 30
    #x = [ random.random() for _ in range(n) ]
    #y = [ random.random() for _ in range(n) ]
    
    #plt.plot(x,y,"o")
    
    x = [-1]
    y = [1]
    n = len(x)
    for l in balls:
        x.append(l[0])
        y.append(l[1])
    print(x, y)

    ordre = list(range(len(x)))
    print("longueur initiale", longueur(x,y,ordre))

    ordre = n_permutation (x,y, 20)
    print("ordre = ", ordre)
    print("longueur min", longueur(x,y,ordre))
    xo = [ x[o] for o in ordre + [ordre[0]]]
    yo = [ y[o] for o in ordre + [ordre[0]]]
    plt.plot(xo,yo, "o-")

    #plt.text(xo[0],yo[0],"0",color="r",weight="bold",size="x-large")
    #plt.text(xo[-2],yo[-2],"N-1",color="r",weight="bold",size="x-large")


    while True:
        show_img("Balls", ball_img)
        show_img("Zones", zone_img)
        show_img("Origin", img)
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
        break

    plt.show()
