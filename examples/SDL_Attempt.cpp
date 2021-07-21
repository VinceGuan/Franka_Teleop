//
// Created by yuan on 16/07/2021.
//
#include <SDL2/SDL.h>
#include <iostream>

SDL_Event Event;
bool isRunning = true;

void Exit(SDL_Window &window)
{
  SDL_DestroyWindow(&window);
  SDL_Quit();
}


//int main (int argc, char *argv[])
int main ()
{
  //SDL_Init(SDL_INIT_EVERYTHING);
  //SDL_Quit();

  // Print SDL information
  SDL_version compiled;
  SDL_version linked;

  SDL_VERSION(&compiled);
  SDL_GetVersion(&linked);
  printf("We compiled against SDL version %d.%d.%d ...\n",
         compiled.major, compiled.minor, compiled.patch);
  printf("But we are linking against SDL version %d.%d.%d.\n",
         linked.major, linked.minor, linked.patch);


  SDL_Window *window = nullptr;

  if(SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "SDL Video Initialisation Error: " << SDL_GetError() << std::endl;
  }
  else
  {
    window = SDL_CreateWindow("A Window Title",SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,640,480,SDL_WINDOW_SHOWN);
    if (window == nullptr) {
      std::cout << "SDL Window Initialisation Error: " << SDL_GetError() << std::endl;
    }
    else {
      SDL_UpdateWindowSurface(window);
      //SDL_Delay(2000);
    }
  }

  ////////////// <START> Joystick Init ////////////
  SDL_Init(SDL_INIT_JOYSTICK);
  if (SDL_NumJoysticks() < 1) {
    std::cout << "No Joystick found." << std::endl;
  }
  SDL_Joystick *joystick = SDL_JoystickOpen(0);
  if (joystick == NULL) {
    std::cout << "Joystick 0 not initialised" << std::endl;
  }
  else {std::cout << "Joystick 0 initialised" << std::endl;}

  SDL_Haptic *haptic = SDL_HapticOpen(1);
  SDL_HapticRumblePlay( haptic, 0.5, 2000 );
  ////////////// <END> Joystick Init ////////////


  while (isRunning) {
    ////////////// <START> Joystick Events ////////////
    while (SDL_PollEvent(&Event) != 0) {
      if (Event.type == SDL_KEYDOWN) {
        switch (Event.key.keysym.sym) {
          case SDLK_q: isRunning = false; break;
        }
      }
      else if (Event.type == SDL_JOYAXISMOTION) {
        if (Event.jaxis.axis == 5) {
            std::cout << Event.jaxis.value << std::endl;
          }
        else if (Event.jaxis.axis == 1) {
          std::cout << Event.jaxis.value << std::endl;
          }
        }
      else if (Event.type == SDL_JOYBUTTONDOWN) {
          if (Event.jbutton.button == 0) {
            std::cout << Event.jbutton.button << std::endl;
        }
      }
    }
    /////////////// <END> Joystick Events /////////////
  }

  Exit(*window);

  return 0;
}