#ifndef OPENAL_HH
#define OPENAL_HH

#ifndef DISABLE_OPENAL
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#endif 

#include <stdint.h>

#include <deque>

#include "SingletonT.hh"

namespace gazebo
{
  
  class OpenAL : public SingletonT<OpenAL>
  {
    /// \brief Constructor
    private: OpenAL();
  
    /// \brief Destructor
    private: virtual ~OpenAL();
  
    /// \brief Initialize
    public: int Init();
  
    /// \brief Update all the sources and the listener
    public: void Update();
  
    /// \brief Create a sound source
    public: unsigned int CreateSource();
  
    /// \brief Create an audio data buffer
    public: unsigned int CreateBuffer(const std::string &audioFile);
  
    /// \brief Set the data
    public: int SetData(unsigned int index, uint8_t *data, unsigned int dataSize, unsigned int freq);
  
    /// \brief Attach a buffer to a source
    public: int SetSourceBuffer(unsigned int sourceIndex, unsigned int bufferIndex);
  
    /// \brief Play a sound
    public: void Play(unsigned int source);
  
    /// \brief Set the listener position
    public: int SetListenerPos( float x, float y, float z );
  
    /// \brief Set the listener velocity
    public: int SetListenerVel( float x, float y, float z );
  
    /// \brief Set the listener orientation
    public: int SetListenerOrient(float cx, float cy, float cz,
                                        float ux, float uy, float uz);
  
    /// \brief Set the position of the source
    public: int SetSourcePos(unsigned int index, float x, float y, float z);
  
    /// \brief Set the position of the source
    public: int SetSourceVel(unsigned int index, float x, float y, float z);
  
    /// \brief Set the pitch of the source
    public: int SetSourcePitch(unsigned int index, float p);
  
    /// \brief Set the pitch of the source
    public: int SetSourceGain(unsigned int index, float g);
  
    /// \brief Set whether the source loops the audio 
    public: int SetSourceLoop(unsigned int index, bool state);

#ifndef DISABLE_OPENAL
    private: ALCcontext *context;
    private: ALCdevice *audioDevice;
  
    private: ALfloat pos[3];

    // OpenAL error code
    private: ALenum error;
#endif

    // Audio sources. 
    private: std::deque<unsigned int> sources;
  
    // Audio data buffers
    private: std::deque<unsigned int> buffers;
  
    private: friend class DestroyerT<OpenAL>;
    private: friend class SingletonT<OpenAL>;
  };
  
}

#endif
