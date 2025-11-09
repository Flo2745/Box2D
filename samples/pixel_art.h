#pragma once

#include "box2d/box2d.h"

#include <cstdint>
#include <string>
#include <vector>

/// Représente un motif de pixel art couleur arbitraire (un body, plusieurs fixtures)
struct PixelArtColor
{
	int width = 0;
	int height = 0;
	std::vector<uint32_t> pixels; // Couleur par pixel (0 = transparent sinon 0xRRGGBB ou 0xAARRGGBB)

	PixelArtColor() = default;
	PixelArtColor( int w, int h )
		: width( w )
		, height( h )
		, pixels( w * h, 0 )
	{
	}

	uint32_t at( int x, int y ) const
	{
		return pixels[y * width + x];
	}
	uint32_t& at( int x, int y )
	{
		return pixels[y * width + x];
	}
};

/// --- Ajout pour la gestion des contours physiques/sensors ---
/// Type de pixel dans la map physique
enum PixelPhysicsType
{
	Pixel_Void = 0, // Transparent ou vide
	Pixel_Physical, // Contour = collision physique
	Pixel_Sensor	// Intérieur = sensor
};

/// Calcule la carte de physique par pixel : contour = physique, intérieur = sensor
void ComputePixelPhysicsMap( const PixelArtColor& art, std::vector<PixelPhysicsType>& outMap );

/// Crée un body où chaque pixel coloré du contour est une fixture physique, et chaque pixel intérieur est sensor.
/// (Tu peux surcharger cette fonction pour mettre des params friction/restitution si tu veux)
b2BodyId CreatePixelArtBody_PhysicsContour( b2WorldId worldId, const PixelArtColor& art, float pixelSize = 1.0f,
											b2Vec2 origin = { 0, 0 }, float linearDamping = 2.0f, float angularDamping = 2.0f,
											float restitution = 0.1f, float friction = 0.6f );

/// Enregistre un motif couleur (appelé en statique dans pixel_art.cpp)
void PixelArtColor_Register( const std::string& name, int w, int h, const uint32_t* data );

/// Récupère un motif par son nom, nullptr si absent
const PixelArtColor* PixelArtColor_GetByName( const std::string& name );

/// Liste tous les noms de motifs couleur disponibles
std::vector<std::string> PixelArtColor_GetAllNames();

/// Renvoie un motif par son index dans la liste des noms (nullptr si invalide)
const PixelArtColor* PixelArtColor_GetByIndex( size_t idx );

/// Crée un body unique, positionné à `origin`, avec une fixture par pixel coloré (version full physique)
b2BodyId CreatePixelArtBody( b2WorldId worldId, const PixelArtColor& art, float pixelSize = 1.0f, b2Vec2 origin = { 0, 0 },
							 float linearDamping = 2.0f, float angularDamping = 2.0f, float restitution = 0.1f,
							 float friction = 0.6f );

/// Convertit un tableau d'indices palette (uint8_t) + palette en PixelArtColor prêt à enregistrer.
/// indices doit être de taille w*h. palette doit contenir paletteSize couleurs.
void MakePixelArtColorFromPalette( int w, int h, const uint8_t* indices, const uint32_t* palette, size_t paletteSize,
								   PixelArtColor& out );

/// Récupère les 8 points {col, row} définissant le contour polygonal de la tête de la Diamond Axe.
/// `count` est rempli avec le nombre de points (toujours 8 ici).
const int ( *PixelArt_GetDiamondAxeHeadPoints( size_t& count ) )[2];
