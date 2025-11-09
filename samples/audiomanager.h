#pragma once

#include "Draw.h" // NE PAS ENLEVER !

#include "box2d/box2d.h" // Ou adapte selon ton projet

#include <SFML/Audio.hpp>
#include <queue>
#include <string>
#include <vector>

struct HitEvent
{
	b2Vec2 point;
	float speed;
	int stepIndex;
};

class AudioManager
{
public:
	AudioManager();

	// Charge un dossier principal (reset tout)
	void LoadFromDirectory( const std::string& path );
	// Ajoute les sons d’un dossier supplémentaire (append)
	void AddFromDirectory( const std::string& path );
	size_t GetSoundCount() const
	{
		return m_sounds.size();
	}

	void SetVolume( float volume );

	void QueueSound( size_t index );
	void PlayQueued();
	void PlayImmediate( size_t index );

	// -------- NOUVEAU : Système de banques --------

	// Définit la banque courante (index 0 = première banque chargée)
	void SetCurrentBank( size_t bankIndex );
	size_t GetBankCount() const;
	size_t GetCurrentBank() const;
	size_t GetBankSoundCount( size_t bankIndex ) const;

	// Lecture cyclique stricte (séquentielle) de la banque courante
	void PlayNextInBank();

	// Lecture aléatoire dans la banque courante
	void PlayRandomInBank();

	// Son aléatoire (toutes banques confondues, pour compatibilité)
	void PlaySpawnSound();

	// Permet de jouer dans un sous-ensemble (plage d’indices)
	void PlayFromRange( size_t start, size_t count );

	void Clear();

	// Pour tes effets visuels + audio synchronisés
	void HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex );
	void DrawHitEffects( Draw* draw, int currentStep );

	// Helper pour accès multi-dossiers (accès index début d'une banque)
	size_t GetStartIndex( int bank ) const
	{
		return ( bank >= 0 && bank < (int)m_bankOffsets.size() ) ? m_bankOffsets[bank] : 0;
	}

private:
	std::vector<sf::SoundBuffer> m_buffers;
	std::vector<sf::Sound> m_sounds;
	float m_volume = 40.0f;

	std::queue<size_t> m_queue;
	size_t m_nextPlaySlot = 0; // Pour slots audio (round-robin sur les objets sf::Sound)

	// Banques
	std::vector<size_t> m_bankOffsets;	 // index début de chaque banque
	std::vector<size_t> m_bankSizes;	 // nombre de sons par banque
	std::vector<size_t> m_bankNextIndex; // index round-robin dans chaque banque
	size_t m_currentBank = 0;

	// Hit effects
	std::vector<HitEvent> m_hitEvents;
	b2Vec2 m_lastImpactPosition;
	int m_lastImpactStep = -1000;

	static constexpr int MIN_STEPS_BETWEEN_IMPACTS = 8;
	static constexpr float MIN_DISTANCE_BETWEEN_IMPACTS = 0.7f;
};
