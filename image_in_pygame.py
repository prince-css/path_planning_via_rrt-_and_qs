import pygame
pygame.init()
WHITE=(255,255,255)
WIDTH=800
WIN=pygame.display.set_mode((WIDTH,WIDTH))
pygame.display.set_caption("path finding algoriithm")
image = pygame.image.load(r'C:\My_project\cartesian_space(sq)cropped.jpg') 
while True :
	WIN.fill(WHITE)
	WIN.blit(pygame.transform.scale(image, (800, 800)),(0,0))

	for event in pygame.event.get():
		if event.type==pygame.QUIT:
			pygame.quit()
			quit()
		pygame.display.update()