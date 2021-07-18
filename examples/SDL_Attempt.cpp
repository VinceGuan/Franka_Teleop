//
// Created by yuan on 16/07/2021.
//
#include <SDL.h>
#include <iostream>

SDL_Event Event;
bool isRunning = true;

void Exit(SDL_Window &window)
{
  SDL_DestroyWindow(&window);
  SDL_Quit();
}

int main (int argc, char *argv[])
{
  //SDL_Init(SDL_INIT_EVERYTHING);
  //SDL_Quit();

  SDL_Window *window = nullptr;

  if(SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "SDL Video Initialisation Error: " << SDL_GetError << std::endl;
  }
  else
  {
    window = SDL_CreateWindow("A Window Title",SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,640,480,SDL_WINDOW_SHOWN);
    if (window == NULL) {
      std::cout << "SDL Window Initialisation Error: " << SDL_GetError << std::endl;
    }
    else {
      SDL_UpdateWindowSurface(window);
      SDL_Delay(2000);
    }
  }

  while (isRunning) {
    while (SDL_PollEvent(&Event) != 0) {
      if (Event.type == SDL_KEYDOWN) {
        switch (Event.key.keysym.sym) {
          case SDLK_q: isRunning = false; break;
        }
      }
    }
  }

  Exit(*window);

  return 0;
}