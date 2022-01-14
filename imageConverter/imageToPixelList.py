from PIL import Image
import numpy
from itertools import chain

def imageToList(img: Image):
    imageNumpyArray = numpy.array(img)
    imagePixelList = imageNumpyArray.tolist()
    imageFlattenedList = list(chain(*imagePixelList))
    return imageFlattenedList


def decreaseBrightnessBy(pixelList: list, decreasmentFactor: int) -> list:
    return [[color // decreasmentFactor for color in pixel] for pixel in pixelList]


def writeToFile(destinationFilePath: str, imageName: str, imageList: list):
    file = open(f"{destinationFilePath}", "a")
    file.write(f"{imageName} [256][3] = {{")
    for pixelColors in imageList:
        file.write(f"{{ {','.join(list(map(str, pixelColors)))} }}, \n")
    file.write("};\n")
    file.close()


def imageResizeAndAdapt(img: str) -> list:
    img = Image.open(img)
    img = img.convert("RGB") if img.mode in ("RGBA") else img  # convert .png to .jpeg to ommit transparent parts of it


    # resize image using nearest-neighbor compression to fit in a 16x16 matrix
    imgSmall = img.resize((16, 16), resample=Image.NEAREST)
    imgSmall.save("images/resized_near.png")

    imgFirstHalf = imgSmall.crop((0, 0, 8, 16))

    # Due to the fact that we connected our matrices in a weird way, to display an image on the top 4 matrices,
    # the right half of the image should be flipped both horizontally and vertically.
    imgSecondHalf = imgSmall.crop((8, 0, 16, 16))
    flippedSecondHalf = imgSecondHalf.transpose(Image.FLIP_TOP_BOTTOM).transpose(Image.FLIP_LEFT_RIGHT)

    return imgFirstHalf, flippedSecondHalf


if __name__ == "__main__":
    # souceImagePath = input("Write path to the source image: ")
    souceImagePath ="./images/curve_left.png"

    resultImage = imageResizeAndAdapt(souceImagePath)
    resultPixelList = imageToList(resultImage[0]) + imageToList(resultImage[1])
    dimmedPixelList = decreaseBrightnessBy(resultPixelList, 3)

    imageName = souceImagePath.split(".")[-2].split("/")[-1]
    # destinationPath = input("Where would you like to save the array of pixels: ")
    destinationPath = "../Core/Inc/additional_signs.h"
    writeToFile(destinationPath, imageName, resultPixelList)
