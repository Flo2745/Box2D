#include "audiomanager.h"

#include <SFML/Audio.hpp>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>

int NaturalCompare( const std::string& a, const std::string& b )
{
	size_t ia = 0, ib = 0;
	while ( ia < a.size() && ib < b.size() )
	{
		if ( std::isdigit( a[ia] ) && std::isdigit( b[ib] ) )
		{
			size_t enda = ia;
			while ( enda < a.size() && std::isdigit( a[enda] ) )
				enda++;
			size_t endb = ib;
			while ( endb < b.size() && std::isdigit( b[endb] ) )
				endb++;

			int na = std::stoi( a.substr( ia, enda - ia ) );
			int nb = std::stoi( b.substr( ib, endb - ib ) );
			if ( na != nb )
				return na - nb;
			ia = enda;
			ib = endb;
		}
		else if ( std::isdigit( a[ia] ) )
		{
			return 1;
		}
		else if ( std::isdigit( b[ib] ) )
		{
			return -1;
		}
		else
		{
			if ( a[ia] != b[ib] )
				return (unsigned char)a[ia] - (unsigned char)b[ib];
			ia++;
			ib++;
		}
	}
	return (int)a.size() - (int)b.size();
}

std::vector<std::filesystem::path> GetSortedFiles( const std::string& path )
{
	std::vector<std::filesystem::path> files;
	for ( const auto& entry : std::filesystem::directory_iterator( path ) )
	{
		if ( entry.is_regular_file() && entry.path().extension() == ".wav" )
			files.push_back( entry.path() );
	}
	auto naturalOrder = []( const std::filesystem::path& a, const std::filesystem::path& b ) {
		return NaturalCompare( a.filename().string(), b.filename().string() ) < 0;
	};
	std::sort( files.begin(), files.end(), naturalOrder );
	return files;
}

AudioManager::AudioManager()
	: m_volume( 40.0f )
	, m_nextPlaySlot( 0 )
	, m_currentBank( 0 )
	, m_lastImpactStep( -1000 )
	, m_lastImpactPosition{ 0, 0 }
{
	m_hitEvents.resize( 8 );
	m_bankOffsets.push_back( 0 );
	m_bankSizes.push_back( 0 );
	m_bankNextIndex.push_back( 0 );
}

void AudioManager::LoadFromDirectory( const std::string& path )
{
	Clear();

	auto files = GetSortedFiles( path );

	std::cout << "Ordre des sons chargés depuis " << path << " :" << std::endl;
	for ( size_t i = 0; i < files.size(); ++i )
		std::cout << "[" << i << "] " << files[i].filename().string() << std::endl;

	if ( files.empty() )
	{
		m_bankOffsets[0] = 0;
		m_bankSizes[0] = 0;
		m_bankNextIndex[0] = 0;
		return;
	}

	m_bankOffsets[0] = 0;
	m_bankSizes[0] = files.size();
	m_bankNextIndex[0] = 0;

	for ( const auto& file : files )
	{
		sf::SoundBuffer buffer;
		if ( !buffer.loadFromFile( file.string() ) )
			throw std::runtime_error( "Cannot load sound: " + file.string() );
		m_buffers.push_back( buffer );
		m_sounds.emplace_back( buffer );
		m_sounds.back().setVolume( m_volume );
	}
}

void AudioManager::AddFromDirectory( const std::string& path )
{
	auto files = GetSortedFiles( path );

	std::cout << "Ajout des sons du dossier : " << path << std::endl;
	for ( size_t i = 0; i < files.size(); ++i )
		std::cout << "[" << ( m_buffers.size() + i ) << "] " << files[i].filename().string() << std::endl;

	if ( files.empty() )
		return;

	size_t offset = m_buffers.size();
	m_bankOffsets.push_back( offset );
	m_bankSizes.push_back( files.size() );
	m_bankNextIndex.push_back( 0 );

	for ( const auto& file : files )
	{
		sf::SoundBuffer buffer;
		if ( !buffer.loadFromFile( file.string() ) )
			throw std::runtime_error( "Cannot load sound: " + file.string() );
		m_buffers.push_back( buffer );
		m_sounds.emplace_back( buffer );
		m_sounds.back().setVolume( m_volume );
	}
}

void AudioManager::SetVolume( float volume )
{
	m_volume = volume;
	for ( auto& s : m_sounds )
		s.setVolume( volume );
}

// -------- NOUVEAU : API de banques --------

void AudioManager::SetCurrentBank( size_t bankIndex )
{
	if ( bankIndex < m_bankOffsets.size() )
		m_currentBank = bankIndex;
}

size_t AudioManager::GetBankCount() const
{
	return m_bankOffsets.size();
}

size_t AudioManager::GetCurrentBank() const
{
	return m_currentBank;
}

size_t AudioManager::GetBankSoundCount( size_t bankIndex ) const
{
	if ( bankIndex < m_bankSizes.size() )
		return m_bankSizes[bankIndex];
	return 0;
}

// ----------- LECTURE -----------

// Lecture séquentielle stricte dans la banque courante (round-robin)
void AudioManager::PlayNextInBank()
{
	if ( m_bankSizes[m_currentBank] == 0 )
		return;
	size_t start = m_bankOffsets[m_currentBank];
	size_t count = m_bankSizes[m_currentBank];
	size_t& nextIndex = m_bankNextIndex[m_currentBank];

	size_t idx = start + nextIndex;
	PlayImmediate( idx );
	nextIndex = ( nextIndex + 1 ) % count;
}

// Lecture aléatoire dans la banque courante
void AudioManager::PlayRandomInBank()
{
	size_t count = m_bankSizes[m_currentBank];
	if ( count == 0 )
		return;
	size_t start = m_bankOffsets[m_currentBank];
	static thread_local std::mt19937 rng{ std::random_device{}() };
	size_t idx = start + ( std::uniform_int_distribution<size_t>( 0, count - 1 ) )( rng );
	PlayImmediate( idx );
}

// Lecture immédiate (utilisé par tout le reste)
void AudioManager::PlayImmediate( size_t index )
{
	if ( index >= m_sounds.size() )
		return;

	// Slot audio round-robin (évite d’écraser un son déjà en lecture)
	for ( size_t i = 0; i < m_sounds.size(); ++i )
	{
		size_t playIdx = ( m_nextPlaySlot + i ) % m_sounds.size();
		if ( m_sounds[playIdx].getStatus() != sf::Sound::Status::Playing )
		{
			m_sounds[playIdx].setBuffer( m_buffers[index] );
			m_sounds[playIdx].setVolume( m_volume );
			m_sounds[playIdx].play();
			m_nextPlaySlot = ( playIdx + 1 ) % m_sounds.size();
			return;
		}
	}
	// Si tout est occupé, on force sur m_nextPlaySlot
	m_sounds[m_nextPlaySlot].setBuffer( m_buffers[index] );
	m_sounds[m_nextPlaySlot].setVolume( m_volume );
	m_sounds[m_nextPlaySlot].play();
	m_nextPlaySlot = ( m_nextPlaySlot + 1 ) % m_sounds.size();
}

// API queue (optionnel, pour jouer dans l'ordre qu'on veut)
void AudioManager::QueueSound( size_t index )
{
	if ( index < m_sounds.size() )
		m_queue.push( index );
}

void AudioManager::PlayQueued()
{
	while ( !m_queue.empty() )
	{
		PlayImmediate( m_queue.front() );
		m_queue.pop();
	}
}

// -------- API pour events --------

// Joue le prochain son séquentiel de la banque courante à chaque impact
void AudioManager::HandleHitEffect( const b2Vec2& impactPoint, float speed, int stepIndex )
{
	m_lastImpactPosition = impactPoint;
	m_lastImpactStep = stepIndex;

	HitEvent* oldestEvent = &m_hitEvents[0];
	for ( auto& event : m_hitEvents )
	{
		if ( event.stepIndex < oldestEvent->stepIndex )
			oldestEvent = &event;
	}

	oldestEvent->point = impactPoint;
	oldestEvent->speed = speed;
	oldestEvent->stepIndex = stepIndex;

	PlayNextInBank(); // Pas de filtrage temporel ou spatial
}

// Pour les autres events (spawn etc)
void AudioManager::PlaySpawnSound()
{
	PlayRandomInBank();
}

void AudioManager::PlayFromRange( size_t start, size_t count )
{
	if ( count == 0 || start >= m_sounds.size() )
		return;
	static thread_local std::mt19937 rng{ std::random_device{}() };
	size_t idx = start + ( std::uniform_int_distribution<size_t>( 0, count - 1 ) )( rng );
	PlayImmediate( idx );
}

void AudioManager::Clear()
{
	m_buffers.clear();
	m_sounds.clear();
	m_bankOffsets.clear();
	m_bankSizes.clear();
	m_bankNextIndex.clear();
	std::queue<size_t> empty;
	std::swap( m_queue, empty );
	m_nextPlaySlot = 0;
	m_currentBank = 0;

	// Toujours garder une banque par défaut (même vide)
	m_bankOffsets.push_back( 0 );
	m_bankSizes.push_back( 0 );
	m_bankNextIndex.push_back( 0 );
}
