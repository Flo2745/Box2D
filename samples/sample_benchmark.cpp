// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

// ======= Project headers =======
#include "audiomanager.h"
#include "benchmarks.h"
#include "draw.h"
#include "human.h"
#include "random.h"
#include "sample.h"
#include "pixel_art.h"

// ======= Box2D =======
#include "box2d/box2d.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

// ======= Third-party / system =======
#include <GLFW/glfw3.h>
#include <SFML/Audio.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <imgui.h>
#include <iostream>
#include <limits>
#include <list>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#if defined( _MSC_VER )
#include <intrin.h>
#define GET_CYCLES __rdtsc()
#else
#define GET_CYCLES b2GetTicks()
#endif

// ======= b2BodyId helpers / hashing =======
namespace std
{
template <> struct hash<b2BodyId>
{
	std::size_t operator()( const b2BodyId& id ) const noexcept
	{
		return std::hash<uint64_t>{}( b2StoreBodyId( id ) );
	}
};
} // namespace std

inline bool operator==( const b2BodyId& a, const b2BodyId& b )
{
	return b2StoreBodyId( a ) == b2StoreBodyId( b );
}

inline bool operator<( b2BodyId a, b2BodyId b )
{
	const uint64_t ua = b2StoreBodyId( a );
	const uint64_t ub = b2StoreBodyId( b );
	return ua < ub;
}

struct BodyIdHash
{
	std::size_t operator()( const b2BodyId& id ) const noexcept
	{
		return std::hash<uint64_t>{}( b2StoreBodyId( id ) );
	}
};

constexpr float b2_pi = 3.14159265359f;

// Note: resetting the scene is non-deterministic because the world uses freelists
class BenchmarkBarrel : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_mixShape,
		e_compoundShape,
		e_humanShape,
	};

	enum
	{
		e_maxColumns = 26,
		e_maxRows = 150,
	};

	explicit BenchmarkBarrel( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 8.0f, 53.0f };
			m_context->camera.zoom = 25.0f * 2.35f;
		}

		m_context->debugDraw.drawJoints = false;

		{
			float gridSize = 1.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				x += gridSize;
			}

			y = gridSize;
			x = -40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				y += gridSize;
			}

			y = gridSize;
			x = 40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				y += gridSize;
			}

			b2Segment segment = { { -800.0f, -80.0f }, { 800.0f, -80.f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		memset( m_humans, 0, sizeof( m_humans ) );

		m_shapeType = e_compoundShape;

		CreateScene();
	}

	void CreateScene()
	{
		g_randomSeed = 42;

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}

			if ( m_humans[i].isSpawned )
			{
				DestroyHuman( m_humans + i );
			}
		}

		m_columnCount = m_isDebug ? 10 : e_maxColumns;
		m_rowCount = m_isDebug ? 40 : e_maxRows;

		if ( m_shapeType == e_compoundShape )
		{
			if constexpr ( m_isDebug == false )
			{
				m_columnCount = 20;
			}
		}
		else if ( m_shapeType == e_humanShape )
		{
			if constexpr ( m_isDebug )
			{
				m_rowCount = 5;
				m_columnCount = 10;
			}
			else
			{
				m_rowCount = 30;
			}
		}

		float rad = 0.5f;

		float shift = 1.15f;
		float centerx = shift * m_columnCount / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		// todo eliminate this once rolling resistance is added
		if ( m_shapeType == e_mixShape )
		{
			bodyDef.angularDamping = 0.3f;
		}

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		b2Capsule capsule = { { 0.0f, -0.25f }, { 0.0f, 0.25f }, rad };
		b2Circle circle = { { 0.0f, 0.0f }, rad };

		b2Vec2 points[3] = { { -0.1f, -0.5f }, { 0.1f, -0.5f }, { 0.0f, 0.5f } };
		b2Hull wedgeHull = b2ComputeHull( points, 3 );
		b2Polygon wedge = b2MakePolygon( &wedgeHull, 0.0f );

		b2Vec2 vertices[3];
		vertices[0] = { -1.0f, 0.0f };
		vertices[1] = { 0.5f, 1.0f };
		vertices[2] = { 0.0f, 2.0f };
		b2Hull hull = b2ComputeHull( vertices, 3 );
		b2Polygon left = b2MakePolygon( &hull, 0.0f );

		vertices[0] = { 1.0f, 0.0f };
		vertices[1] = { -0.5f, 1.0f };
		vertices[2] = { 0.0f, 2.0f };
		hull = b2ComputeHull( vertices, 3 );
		b2Polygon right = b2MakePolygon( &hull, 0.0f );

		// b2Polygon top = b2MakeOffsetBox(0.8f, 0.2f, {0.0f, 0.8f}, 0.0f);
		// b2Polygon leftLeg = b2MakeOffsetBox(0.2f, 0.5f, {-0.6f, 0.5f}, 0.0f);
		// b2Polygon rightLeg = b2MakeOffsetBox(0.2f, 0.5f, {0.6f, 0.5f}, 0.0f);

		float side = -0.1f;
		float extray = 0.5f;

		if ( m_shapeType == e_compoundShape )
		{
			extray = 0.25f;
			side = 0.25f;
			shift = 2.0f;
			centerx = shift * m_columnCount / 2.0f - 1.0f;
		}
		else if ( m_shapeType == e_humanShape )
		{
			extray = 0.5f;
			side = 0.55f;
			shift = 2.5f;
			centerx = shift * m_columnCount / 2.0f;
		}

		int index = 0;
		float yStart = m_shapeType == e_humanShape ? 2.0f : 100.0f;

		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < m_rowCount; ++j )
			{
				float y = j * ( shift + extray ) + centery + yStart;

				bodyDef.position = { x + side, y };
				side = -side;

				if ( m_shapeType == e_circleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					circle.radius = RandomFloatRange( 0.25f, 0.75f );
					shapeDef.material.rollingResistance = 0.2f;
					b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
				}
				else if ( m_shapeType == e_capsuleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					capsule.radius = RandomFloatRange( 0.25f, 0.5f );
					float length = RandomFloatRange( 0.25f, 1.0f );
					capsule.center1 = { 0.0f, -0.5f * length };
					capsule.center2 = { 0.0f, 0.5f * length };
					shapeDef.material.rollingResistance = 0.2f;
					b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
				}
				else if ( m_shapeType == e_mixShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );

					int mod = index % 3;
					if ( mod == 0 )
					{
						circle.radius = RandomFloatRange( 0.25f, 0.75f );
						b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
					}
					else if ( mod == 1 )
					{
						capsule.radius = RandomFloatRange( 0.25f, 0.5f );
						float length = RandomFloatRange( 0.25f, 1.0f );
						capsule.center1 = { 0.0f, -0.5f * length };
						capsule.center2 = { 0.0f, 0.5f * length };
						b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
					}
					else if ( mod == 2 )
					{
						float width = RandomFloatRange( 0.1f, 0.5f );
						float height = RandomFloatRange( 0.5f, 0.75f );
						b2Polygon box = b2MakeBox( width, height );

						// Don't put a function call into a macro.
						float value = RandomFloatRange( -1.0f, 1.0f );
						box.radius = 0.25f * b2MaxFloat( 0.0f, value );
						b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );
					}
					else
					{
						wedge.radius = RandomFloatRange( 0.1f, 0.25f );
						b2CreatePolygonShape( m_bodies[index], &shapeDef, &wedge );
					}
				}
				else if ( m_shapeType == e_compoundShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );

					b2CreatePolygonShape( m_bodies[index], &shapeDef, &left );
					b2CreatePolygonShape( m_bodies[index], &shapeDef, &right );
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &top);
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &leftLeg);
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &rightLeg);
				}
				else if ( m_shapeType == e_humanShape )
				{
					float scale = 3.5f;
					float jointFriction = 0.05f;
					float jointHertz = 5.0f;
					float jointDamping = 0.5f;
					CreateHuman( m_humans + index, m_worldId, bodyDef.position, scale, jointFriction, jointHertz, jointDamping,
								 index + 1, nullptr, false );
				}

				index += 1;
			}
		}
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 6.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 15.0f * fontSize, height ) );
		ImGui::Begin( "Benchmark: Barrel", nullptr, ImGuiWindowFlags_NoResize );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Capsule", "Mix", "Compound", "Human" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkBarrel( context );
	}

	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	Human m_humans[e_maxRows * e_maxColumns];
	int m_columnCount;
	int m_rowCount;

	ShapeType m_shapeType;
};

static int benchmarkBarrel = RegisterSample( "Benchmark", "Barrel", BenchmarkBarrel::Create );

// This is used to compare performance with Box2D v2.4
class BenchmarkBarrel24 : public Sample
{
public:
	explicit BenchmarkBarrel24( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 8.0f, 53.0f };
			m_context->camera.zoom = 25.0f * 2.35f;
		}

		float groundSize = 25.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( groundSize, 1.2f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			bodyDef.rotation = b2MakeRot( 0.5f * B2_PI );
			bodyDef.position = { groundSize, 2.0f * groundSize };
			groundId = b2CreateBody( m_worldId, &bodyDef );

			box = b2MakeBox( 2.0f * groundSize, 1.2f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			bodyDef.position = { -groundSize, 2.0f * groundSize };
			groundId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		int32_t num = 26;
		float rad = 0.5f;

		float shift = rad * 2.0f;
		float centerx = shift * num / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		b2Polygon cuboid = b2MakeSquare( 0.5f );

		// b2Polygon top = b2MakeOffsetBox(0.8f, 0.2f, {0.0f, 0.8f}, 0.0f);
		// b2Polygon leftLeg = b2MakeOffsetBox(0.2f, 0.5f, {-0.6f, 0.5f}, 0.0f);
		// b2Polygon rightLeg = b2MakeOffsetBox(0.2f, 0.5f, {0.6f, 0.5f}, 0.0f);

#ifdef _DEBUG
		int numj = 5;
#else
		int numj = 5 * num;
#endif
		for ( int i = 0; i < num; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < numj; ++j )
			{
				float y = j * shift + centery + 2.0f;

				bodyDef.position = { x, y };

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &cuboid );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkBarrel24( context );
	}
};

static int benchmarkBarrel24 = RegisterSample( "Benchmark", "Barrel 2.4", BenchmarkBarrel24::Create );

class BenchmarkTumbler : public Sample
{
public:
	explicit BenchmarkTumbler( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.5f, 10.0f };
			m_context->camera.zoom = 15.0f;
		}

		CreateTumbler( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkTumbler( context );
	}
};

static int benchmarkTumbler = RegisterSample( "Benchmark", "Tumbler", BenchmarkTumbler::Create );

class BenchmarkWasher : public Sample
{
public:
	explicit BenchmarkWasher( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.5f, 10.0f };
			m_context->camera.zoom = 20.0f;
		}

		CreateWasher( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkWasher( context );
	}
};

static int benchmarkWasher = RegisterSample( "Benchmark", "Washer", BenchmarkWasher::Create );

// todo try removing kinematics from graph coloring
class BenchmarkManyTumblers : public Sample
{
public:
	explicit BenchmarkManyTumblers( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.0f, -5.5 };
			m_context->camera.zoom = 25.0f * 3.4f;
			m_context->debugDraw.drawJoints = false;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		m_rowCount = m_isDebug ? 2 : 19;
		m_columnCount = m_isDebug ? 2 : 19;

		m_tumblerIds = nullptr;
		m_positions = nullptr;
		m_tumblerCount = 0;

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		m_angularSpeed = 25.0f;

		CreateScene();
	}

	~BenchmarkManyTumblers() override
	{
		free( m_tumblerIds );
		free( m_positions );
		free( m_bodyIds );
	}

	void CreateTumbler( b2Vec2 position, int index )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = { position.x, position.y };
		bodyDef.angularVelocity = ( B2_PI / 180.0f ) * m_angularSpeed;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
		m_tumblerIds[index] = bodyId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { 2.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { -2.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, 2.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, -2.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
			}
		}

		for ( int i = 0; i < m_tumblerCount; ++i )
		{
			b2DestroyBody( m_tumblerIds[i] );
		}

		free( m_tumblerIds );
		free( m_positions );

		m_tumblerCount = m_rowCount * m_columnCount;
		m_tumblerIds = static_cast<b2BodyId*>( malloc( m_tumblerCount * sizeof( b2BodyId ) ) );
		m_positions = static_cast<b2Vec2*>( malloc( m_tumblerCount * sizeof( b2Vec2 ) ) );

		int index = 0;
		float x = -4.0f * m_rowCount;
		for ( int i = 0; i < m_rowCount; ++i )
		{
			float y = -4.0f * m_columnCount;
			for ( int j = 0; j < m_columnCount; ++j )
			{
				m_positions[index] = { x, y };
				CreateTumbler( m_positions[index], index );
				++index;
				y += 8.0f;
			}

			x += 8.0f;
		}

		free( m_bodyIds );

		int bodiesPerTumbler = m_isDebug ? 8 : 50;
		m_bodyCount = bodiesPerTumbler * m_tumblerCount;

		m_bodyIds = static_cast<b2BodyId*>( malloc( m_bodyCount * sizeof( b2BodyId ) ) );

		memset( m_bodyIds, 0, m_bodyCount * sizeof( b2BodyId ) );
		m_bodyIndex = 0;
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 8.5f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 15.5f * fontSize, height ) );
		ImGui::Begin( "Benchmark: Many Tumblers", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 8.0f * fontSize );

		bool changed = false;
		changed = changed || ImGui::SliderInt( "Row Count", &m_rowCount, 1, 32 );
		changed = changed || ImGui::SliderInt( "Column Count", &m_columnCount, 1, 32 );

		if ( changed )
		{
			CreateScene();
		}

		if ( ImGui::SliderFloat( "Speed", &m_angularSpeed, 0.0f, 100.0f, "%.f" ) )
		{
			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				b2Body_SetAngularVelocity( m_tumblerIds[i], ( B2_PI / 180.0f ) * m_angularSpeed );
				b2Body_SetAwake( m_tumblerIds[i], true );
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		if ( m_bodyIndex < m_bodyCount && ( m_stepCount & 0x7 ) == 0 )
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Capsule capsule = { { -0.1f, 0.0f }, { 0.1f, 0.0f }, 0.075f };

			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				assert( m_bodyIndex < m_bodyCount );

				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = m_positions[i];
				m_bodyIds[m_bodyIndex] = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( m_bodyIds[m_bodyIndex], &shapeDef, &capsule );

				m_bodyIndex += 1;
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkManyTumblers( context );
	}

	b2BodyId m_groundId;

	int m_rowCount;
	int m_columnCount;

	b2BodyId* m_tumblerIds;
	b2Vec2* m_positions;
	int m_tumblerCount;

	b2BodyId* m_bodyIds;
	int m_bodyCount;
	int m_bodyIndex;

	float m_angularSpeed;
};

static int benchmarkManyTumblers = RegisterSample( "Benchmark", "Many Tumblers", BenchmarkManyTumblers::Create );

class BenchmarkLargePyramid : public Sample
{
public:
	explicit BenchmarkLargePyramid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 50.0f };
			m_context->camera.zoom = 25.0f * 2.2f;
			m_context->enableSleep = false;
		}

		CreateLargePyramid( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkLargePyramid( context );
	}
};

static int benchmarkLargePyramid = RegisterSample( "Benchmark", "Large Pyramid", BenchmarkLargePyramid::Create );

class BenchmarkManyPyramids : public Sample
{
public:
	explicit BenchmarkManyPyramids( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 16.0f, 110.0f };
			m_context->camera.zoom = 25.0f * 5.0f;
			m_context->enableSleep = false;
		}

		CreateManyPyramids( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkManyPyramids( context );
	}
};

static int benchmarkManyPyramids = RegisterSample( "Benchmark", "Many Pyramids", BenchmarkManyPyramids::Create );

class BenchmarkCreateDestroy : public Sample
{
public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * ( e_maxBaseCount + 1 ) / 2
	};

	explicit BenchmarkCreateDestroy( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 50.0f };
			m_context->camera.zoom = 25.0f * 2.2f;
		}

		float groundSize = 100.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( groundSize, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		m_baseCount = m_isDebug ? 40 : 100;
		m_iterations = m_isDebug ? 1 : 10;
		m_bodyCount = 0;
	}

	void CreateScene()
	{
		uint64_t ticks = b2GetTicks();

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}
		}

		m_destroyTime += b2GetMillisecondsAndReset( &ticks );

		int count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		float h = 0.5f;
		b2Polygon box = b2MakeRoundedBox( h, h, 0.0f );

		int index = 0;

		for ( int i = 0; i < count; ++i )
		{
			float y = i * shift + centery;

			for ( int j = i; j < count; ++j )
			{
				float x = 0.5f * i * shift + ( j - i ) * shift - centerx;
				bodyDef.position = { x, y };

				assert( index < e_maxBodyCount );
				m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );

				index += 1;
			}
		}

		m_createTime += b2GetMilliseconds( ticks );

		m_bodyCount = index;

		b2World_Step( m_worldId, 1.0f / 60.0f, 4 );
	}

	void Step() override
	{
		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		for ( int i = 0; i < m_iterations; ++i )
		{
			CreateScene();
		}

		DrawTextLine( "total: create = %g ms, destroy = %g ms", m_createTime, m_destroyTime );

		float createPerBody = 1000.0f * m_createTime / m_iterations / m_bodyCount;
		float destroyPerBody = 1000.0f * m_destroyTime / m_iterations / m_bodyCount;
		DrawTextLine( "body: create = %g us, destroy = %g us", createPerBody, destroyPerBody );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCreateDestroy( context );
	}

	float m_createTime;
	float m_destroyTime;
	b2BodyId m_bodies[e_maxBodyCount];
	int m_bodyCount;
	int m_baseCount;
	int m_iterations;
};

static int benchmarkCreateDestroy = RegisterSample( "Benchmark", "CreateDestroy", BenchmarkCreateDestroy::Create );

class BenchmarkSleep : public Sample
{
public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * ( e_maxBaseCount + 1 ) / 2
	};

	explicit BenchmarkSleep( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 50.0f };
			m_context->camera.zoom = 25.0f * 2.2f;
		}

		{
			float groundSize = 100.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( groundSize, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_baseCount = m_isDebug ? 40 : 100;
		m_bodyCount = 0;

		int count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		float h = 0.5f;
		b2Polygon box = b2MakeRoundedBox( h, h, 0.0f );

		int index = 0;

		for ( int i = 0; i < count; ++i )
		{
			float y = i * shift + centery;

			for ( int j = i; j < count; ++j )
			{
				float x = 0.5f * i * shift + ( j - i ) * shift - centerx;
				bodyDef.position = { x, y };

				assert( index < e_maxBodyCount );
				m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );

				index += 1;
			}
		}

		m_bodyCount = index;

		m_wakeTotal = 0.0f;
		m_sleepTotal = 0.0f;
	}

	void Step() override
	{
		// These operations don't show up in b2Profile
		if ( m_stepCount > 20 )
		{
			// Creating and destroying a joint will engage the island splitter.
			b2FilterJointDef jointDef = b2DefaultFilterJointDef();
			jointDef.base.bodyIdA = m_bodies[0];
			jointDef.base.bodyIdB = m_bodies[1];
			b2JointId jointId = b2CreateFilterJoint( m_worldId, &jointDef );

			uint64_t ticks = b2GetTicks();

			// This will wake the island
			b2DestroyJoint( jointId, true );
			m_wakeTotal += b2GetMillisecondsAndReset( &ticks );

			// Put the island back to sleep. It must be split because a constraint was removed.
			b2Body_SetAwake( m_bodies[0], false );
			m_sleepTotal += b2GetMillisecondsAndReset( &ticks );

			int count = m_stepCount - 20;
			DrawTextLine( "wake ave = %g ms", m_wakeTotal / count );
			DrawTextLine( "sleep ave = %g ms", m_sleepTotal / count );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSleep( context );
	}

	b2BodyId m_bodies[e_maxBodyCount];
	int m_bodyCount;
	int m_baseCount;
	float m_wakeTotal;
	float m_sleepTotal;
	bool m_awake;
};

static int benchmarkSleep = RegisterSample( "Benchmark", "Sleep", BenchmarkSleep::Create );

class BenchmarkJointGrid : public Sample
{
public:
	explicit BenchmarkJointGrid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 60.0f, -57.0f };
			m_context->camera.zoom = 25.0f * 2.5f;
			m_context->enableSleep = false;
		}

		CreateJointGrid( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkJointGrid( context );
	}
};

static int benchmarkJointGridIndex = RegisterSample( "Benchmark", "Joint Grid", BenchmarkJointGrid::Create );

class BenchmarkSmash : public Sample
{
public:
	explicit BenchmarkSmash( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 60.0f, 6.0f };
			m_context->camera.zoom = 25.0f * 1.6f;
		}

		CreateSmash( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSmash( context );
	}
};

static int sampleSmash = RegisterSample( "Benchmark", "Smash", BenchmarkSmash::Create );

class BenchmarkCompound : public Sample
{
public:
	explicit BenchmarkCompound( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 18.0f, 115.0f };
			m_context->camera.zoom = 25.0f * 5.5f;
		}

		float grid = 1.0f;
#ifdef NDEBUG
		int height = 200;
		int width = 200;
#else
		int height = 100;
		int width = 100;
#endif
		{

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = grid * j;
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
					b2CreatePolygonShape( groundId, &shapeDef, &square );
				}
			}

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = -grid * j;
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
					b2CreatePolygonShape( groundId, &shapeDef, &square );
				}
			}
		}

		{
#ifdef NDEBUG
			int span = 20;
			int count = 5;
#else
			int span = 5;
			int count = 5;
#endif

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			// defer mass properties to avoid n-squared mass computations
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.updateBodyMass = false;

			for ( int m = 0; m < count; ++m )
			{
				float ybody = ( 100.0f + m * span ) * grid;

				for ( int n = 0; n < count; ++n )
				{
					float xbody = -0.5f * grid * count * span + n * span * grid;
					bodyDef.position = { xbody, ybody };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

					for ( int i = 0; i < span; ++i )
					{
						float y = i * grid;
						for ( int j = 0; j < span; ++j )
						{
							float x = j * grid;
							b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
							b2CreatePolygonShape( bodyId, &shapeDef, &square );
						}
					}

					// All shapes have been added so I can efficiently compute the mass properties.
					b2Body_ApplyMassFromShapes( bodyId );
				}
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCompound( context );
	}
};

static int sampleCompound = RegisterSample( "Benchmark", "Compound", BenchmarkCompound::Create );

class BenchmarkKinematic : public Sample
{
public:
	explicit BenchmarkKinematic( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 150.0f;
		}

		float grid = 1.0f;

#ifdef NDEBUG
		int span = 100;
#else
		int span = 20;
#endif

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.angularVelocity = 1.0f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		shapeDef.filter.maskBits = 2;

		// defer mass properties to avoid n-squared mass computations
		shapeDef.updateBodyMass = false;

		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		for ( int i = -span; i < span; ++i )
		{
			float y = i * grid;
			for ( int j = -span; j < span; ++j )
			{
				float x = j * grid;
				b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );
			}
		}

		// All shapes have been added so I can efficiently compute the mass properties.
		b2Body_ApplyMassFromShapes( bodyId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkKinematic( context );
	}
};

static int sampleKinematic = RegisterSample( "Benchmark", "Kinematic", BenchmarkKinematic::Create );

enum QueryType
{
	e_rayCast,
	e_circleCast,
	e_overlap,
};

class BenchmarkCast : public Sample
{
public:
	explicit BenchmarkCast( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 500.0f, 500.0f };
			m_context->camera.zoom = 25.0f * 21.0f;
			// settings.drawShapes = m_isDebug;
		}

		m_queryType = e_circleCast;
		m_ratio = 5.0f;
		m_grid = 1.0f;
		m_fill = 0.1f;
		m_rowCount = m_isDebug ? 100 : 1000;
		m_columnCount = m_isDebug ? 100 : 1000;
		m_minTime = 1e6f;
		m_drawIndex = 0;
		m_topDown = false;
		m_buildTime = 0.0f;
		m_radius = 0.1f;

		g_randomSeed = 1234;
		int sampleCount = m_isDebug ? 100 : 10000;
		m_origins.resize( sampleCount );
		m_translations.resize( sampleCount );
		float extent = m_rowCount * m_grid;

		// Pre-compute rays to avoid randomizer overhead
		for ( int i = 0; i < sampleCount; ++i )
		{
			b2Vec2 rayStart = RandomVec2( 0.0f, extent );
			b2Vec2 rayEnd = RandomVec2( 0.0f, extent );

			m_origins[i] = rayStart;
			m_translations[i] = rayEnd - rayStart;
		}

		BuildScene();
	}

	void BuildScene()
	{
		g_randomSeed = 1234;
		b2DestroyWorld( m_worldId );
		b2WorldDef worldDef = b2DefaultWorldDef();
		m_worldId = b2CreateWorld( &worldDef );

		uint64_t ticks = b2GetTicks();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = 0.0f;

		for ( int i = 0; i < m_rowCount; ++i )
		{
			float x = 0.0f;

			for ( int j = 0; j < m_columnCount; ++j )
			{
				float fillTest = RandomFloatRange( 0.0f, 1.0f );
				if ( fillTest <= m_fill )
				{
					bodyDef.position = { x, y };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

					float ratio = RandomFloatRange( 1.0f, m_ratio );
					float halfWidth = RandomFloatRange( 0.05f, 0.25f );

					b2Polygon box;
					if ( RandomFloat() > 0.0f )
					{
						box = b2MakeBox( ratio * halfWidth, halfWidth );
					}
					else
					{
						box = b2MakeBox( halfWidth, ratio * halfWidth );
					}

					int category = RandomIntRange( 0, 2 );
					shapeDef.filter.categoryBits = 1 << category;
					if ( category == 0 )
					{
						shapeDef.material.customColor = b2_colorBox2DBlue;
					}
					else if ( category == 1 )
					{
						shapeDef.material.customColor = b2_colorBox2DYellow;
					}
					else
					{
						shapeDef.material.customColor = b2_colorBox2DGreen;
					}

					b2CreatePolygonShape( bodyId, &shapeDef, &box );
				}

				x += m_grid;
			}

			y += m_grid;
		}

		if ( m_topDown )
		{
			b2World_RebuildStaticTree( m_worldId );
		}

		m_buildTime = b2GetMilliseconds( ticks );
		m_minTime = 1e6f;
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 17.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 13.0f * fontSize, height ) );

		ImGui::Begin( "Cast", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 7.5f * fontSize );

		bool changed = false;

		const char* queryTypes[] = { "Ray", "Circle", "Overlap" };
		int queryType = int( m_queryType );
		if ( ImGui::Combo( "Query", &queryType, queryTypes, IM_ARRAYSIZE( queryTypes ) ) )
		{
			m_queryType = QueryType( queryType );
			if ( m_queryType == e_overlap )
			{
				m_radius = 5.0f;
			}
			else
			{
				m_radius = 0.1f;
			}

			changed = true;
		}

		if ( ImGui::SliderInt( "rows", &m_rowCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderInt( "columns", &m_columnCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "fill", &m_fill, 0.0f, 1.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "grid", &m_grid, 0.5f, 2.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "ratio", &m_ratio, 1.0f, 10.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::Checkbox( "top down", &m_topDown ) )
		{
			changed = true;
		}

		if ( ImGui::Button( "Draw Next" ) )
		{
			m_drawIndex = ( m_drawIndex + 1 ) % m_origins.size();
		}

		ImGui::PopItemWidth();
		ImGui::End();

		if ( changed )
		{
			BuildScene();
		}
	}

	struct CastResult
	{
		b2Vec2 point;
		float fraction;
		bool hit;
	};

	static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
	{
		CastResult* result = (CastResult*)context;
		result->point = point;
		result->fraction = fraction;
		result->hit = true;
		return fraction;
	}

	struct OverlapResult
	{
		b2Vec2 points[32];
		int count;
	};

	static bool OverlapCallback( b2ShapeId shapeId, void* context )
	{
		OverlapResult* result = (OverlapResult*)context;
		if ( result->count < 32 )
		{
			b2AABB aabb = b2Shape_GetAABB( shapeId );
			result->points[result->count] = b2AABB_Center( aabb );
			result->count += 1;
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		b2QueryFilter filter = b2DefaultQueryFilter();
		filter.maskBits = 1;
		int hitCount = 0;
		int nodeVisits = 0;
		int leafVisits = 0;
		float ms = 0.0f;
		int sampleCount = (int)m_origins.size();

		if ( m_queryType == e_rayCast )
		{
			uint64_t ticks = b2GetTicks();

			b2RayResult drawResult = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2Vec2 origin = m_origins[i];
				b2Vec2 translation = m_translations[i];

				b2RayResult result = b2World_CastRayClosest( m_worldId, origin, translation, filter );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += result.nodeVisits;
				leafVisits += result.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 p1 = m_origins[m_drawIndex];
			b2Vec2 p2 = p1 + m_translations[m_drawIndex];
			DrawLine( m_context->draw, p1, p2, b2_colorWhite );
			DrawPoint( m_context->draw, p1, 5.0f, b2_colorGreen );
			DrawPoint( m_context->draw, p2, 5.0f, b2_colorRed );
			if ( drawResult.hit )
			{
				DrawPoint( m_context->draw, drawResult.point, 5.0f, b2_colorWhite );
			}
		}
		else if ( m_queryType == e_circleCast )
		{
			uint64_t ticks = b2GetTicks();

			CastResult drawResult = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2ShapeProxy proxy = b2MakeProxy( &m_origins[i], 1, m_radius );
				b2Vec2 translation = m_translations[i];

				CastResult result;
				b2TreeStats traversalResult = b2World_CastShape( m_worldId, &proxy, translation, filter, CastCallback, &result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 p1 = m_origins[m_drawIndex];
			b2Vec2 p2 = p1 + m_translations[m_drawIndex];
			DrawLine( m_context->draw, p1, p2, b2_colorWhite );
			DrawPoint( m_context->draw, p1, 5.0f, b2_colorGreen );
			DrawPoint( m_context->draw, p2, 5.0f, b2_colorRed );
			if ( drawResult.hit )
			{
				b2Vec2 t = b2Lerp( p1, p2, drawResult.fraction );
				DrawCircle( m_context->draw, t, m_radius, b2_colorWhite );
				DrawPoint( m_context->draw, drawResult.point, 5.0f, b2_colorWhite );
			}
		}
		else if ( m_queryType == e_overlap )
		{
			uint64_t ticks = b2GetTicks();

			OverlapResult drawResult = {};
			b2Vec2 extent = { m_radius, m_radius };
			OverlapResult result = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2Vec2 origin = m_origins[i];
				b2AABB aabb = { origin - extent, origin + extent };

				result.count = 0;
				b2TreeStats traversalResult = b2World_OverlapAABB( m_worldId, aabb, filter, OverlapCallback, &result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.count;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 origin = m_origins[m_drawIndex];
			b2AABB aabb = { origin - extent, origin + extent };

			DrawBounds( m_context->draw, aabb, b2_colorWhite );

			for ( int i = 0; i < drawResult.count; ++i )
			{
				DrawPoint( m_context->draw, drawResult.points[i], 5.0f, b2_colorHotPink );
			}
		}

		DrawTextLine( "build time ms = %g", m_buildTime );
		DrawTextLine( "hit count = %d, node visits = %d, leaf visits = %d", hitCount, nodeVisits, leafVisits );
		DrawTextLine( "total ms = %.3f", ms );
		DrawTextLine( "min total ms = %.3f", m_minTime );

		float aveRayCost = 1000.0f * m_minTime / float( sampleCount );
		DrawTextLine( "average us = %.2f", aveRayCost );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCast( context );
	}

	QueryType m_queryType;

	std::vector<b2Vec2> m_origins;
	std::vector<b2Vec2> m_translations;
	float m_minTime;
	float m_buildTime;

	int m_rowCount, m_columnCount;
	int m_updateType;
	int m_drawIndex;
	float m_radius;
	float m_fill;
	float m_ratio;
	float m_grid;
	bool m_topDown;
};

static int sampleCast = RegisterSample( "Benchmark", "Cast", BenchmarkCast::Create );

class BenchmarkSpinner : public Sample
{
public:
	explicit BenchmarkSpinner( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 32.0f };
			m_context->camera.zoom = 42.0f;
		}

		// b2_toiCalls = 0;
		// b2_toiHitCount = 0;

		CreateSpinner( m_worldId );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == 1000 && false )
		{
			// 0.1 : 46544, 25752
			// 0.25 : 5745, 1947
			// 0.5 : 2197, 660
			m_context->pause = true;
		}

		// DrawTextLine( "toi calls, hits = %d, %d", b2_toiCalls, b2_toiHitCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSpinner( context );
	}
};

static int sampleSpinner = RegisterSample( "Benchmark", "Spinner", BenchmarkSpinner::Create );

class BenchmarkRain : public Sample
{
public:
	explicit BenchmarkRain( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 110.0f };
			m_context->camera.zoom = 125.0f;
			m_context->enableSleep = true;
		}

		m_context->debugDraw.drawJoints = false;

		CreateRain( m_worldId );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			StepRain( m_worldId, m_stepCount );
		}

		Sample::Step();

		if ( m_stepCount % 1000 == 0 )
		{
			m_stepCount += 0;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkRain( context );
	}
};

static int benchmarkRain = RegisterSample( "Benchmark", "Rain", BenchmarkRain::Create );

class BenchmarkShapeDistance : public Sample
{
public:
	explicit BenchmarkShapeDistance( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 3.0f;
		}

		{
			b2Vec2 points[8] = {};
			b2Rot q = b2MakeRot( 2.0f * B2_PI / 8.0f );
			b2Vec2 p = { 0.5f, 0.0f };
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = b2RotateVector( q, points[i - 1] );
			}

			b2Hull hull = b2ComputeHull( points, 8 );
			m_polygonA = b2MakePolygon( &hull, 0.0f );
		}

		{
			b2Vec2 points[8] = {};
			b2Rot q = b2MakeRot( 2.0f * B2_PI / 8.0f );
			b2Vec2 p = { 0.5f, 0.0f };
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = b2RotateVector( q, points[i - 1] );
			}

			b2Hull hull = b2ComputeHull( points, 8 );
			m_polygonB = b2MakePolygon( &hull, 0.1f );
		}

		// todo arena
		m_transformAs = (b2Transform*)malloc( m_count * sizeof( b2Transform ) );
		m_transformBs = (b2Transform*)malloc( m_count * sizeof( b2Transform ) );
		m_outputs = (b2DistanceOutput*)calloc( m_count, sizeof( b2DistanceOutput ) );

		g_randomSeed = 42;
		for ( int i = 0; i < m_count; ++i )
		{
			m_transformAs[i] = { RandomVec2( -0.1f, 0.1f ), RandomRot() };
			m_transformBs[i] = { RandomVec2( 0.25f, 2.0f ), RandomRot() };
		}

		m_drawIndex = 0;
		m_minCycles = INT_MAX;
		m_minMilliseconds = FLT_MAX;
	}

	~BenchmarkShapeDistance() override
	{
		free( m_transformAs );
		free( m_transformBs );
		free( m_outputs );
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 5.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 17.0f * fontSize, height ) );
		ImGui::Begin( "Benchmark: Shape Distance", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderInt( "draw index", &m_drawIndex, 0, m_count - 1 );

		ImGui::End();
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			b2DistanceInput input = {};
			input.proxyA = b2MakeProxy( m_polygonA.vertices, m_polygonA.count, m_polygonA.radius );
			input.proxyB = b2MakeProxy( m_polygonB.vertices, m_polygonB.count, m_polygonB.radius );
			input.useRadii = true;
			int totalIterations = 0;

			uint64_t start = b2GetTicks();
			uint64_t startCycles = GET_CYCLES;
			for ( int i = 0; i < m_count; ++i )
			{
				b2SimplexCache cache = {};
				input.transformA = m_transformAs[i];
				input.transformB = m_transformBs[i];
				m_outputs[i] = b2ShapeDistance( &input, &cache, nullptr, 0 );
				totalIterations += m_outputs[i].iterations;
			}
			uint64_t endCycles = GET_CYCLES;

			float ms = b2GetMilliseconds( start );
			m_minCycles = b2MinInt( m_minCycles, int( endCycles - startCycles ) );
			m_minMilliseconds = b2MinFloat( m_minMilliseconds, ms );

			DrawTextLine( "count = %d", m_count );
			DrawTextLine( "min cycles = %d", m_minCycles );
			DrawTextLine( "ave cycles = %g", float( m_minCycles ) / float( m_count ) );
			DrawTextLine( "min ms = %g, ave us = %g", m_minMilliseconds, 1000.0f * m_minMilliseconds / float( m_count ) );
			DrawTextLine( "average iterations = %g", totalIterations / float( m_count ) );
		}

		b2Transform xfA = m_transformAs[m_drawIndex];
		b2Transform xfB = m_transformBs[m_drawIndex];
		b2DistanceOutput output = m_outputs[m_drawIndex];
		DrawSolidPolygon( m_context->draw, xfA, m_polygonA.vertices, m_polygonA.count, m_polygonA.radius, b2_colorBox2DGreen );
		DrawSolidPolygon( m_context->draw, xfB, m_polygonB.vertices, m_polygonB.count, m_polygonB.radius, b2_colorBox2DBlue );
		DrawLine( m_context->draw, output.pointA, output.pointB, b2_colorDimGray );
		DrawPoint( m_context->draw, output.pointA, 10.0f, b2_colorWhite );
		DrawPoint( m_context->draw, output.pointB, 10.0f, b2_colorWhite );
		DrawLine( m_context->draw, output.pointA, output.pointA + 0.5f * output.normal, b2_colorYellow );
		DrawTextLine( "distance = %g", output.distance );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkShapeDistance( context );
	}

	static constexpr int m_count = m_isDebug ? 100 : 10000;
	b2Transform* m_transformAs;
	b2Transform* m_transformBs;
	b2DistanceOutput* m_outputs;
	b2Polygon m_polygonA;
	b2Polygon m_polygonB;
	float m_minMilliseconds;
	int m_drawIndex;
	int m_minCycles;
};

static int benchmarkShapeDistance = RegisterSample( "Benchmark", "Shape Distance", BenchmarkShapeDistance::Create );

struct ShapeUserData
{
	int row;
	bool active;
};

class BenchmarkSensor : public Sample
{
public:
	explicit BenchmarkSensor( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 105.0f };
			m_context->camera.zoom = 125.0f;
		}

		b2World_SetCustomFilterCallback( m_worldId, FilterFcn, this );

		m_activeSensor.row = 0;
		m_activeSensor.active = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		{
			float gridSize = 3.0f;

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			shapeDef.userData = &m_activeSensor;

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				x += gridSize;
			}
		}

		g_randomSeed = 42;

		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;

		float yStart = 10.0f;
		m_filterRow = m_rowCount >> 1;

		for ( int j = 0; j < m_rowCount; ++j )
		{
			m_passiveSensors[j].row = j;
			m_passiveSensors[j].active = false;
			shapeDef.userData = m_passiveSensors + j;

			if ( j == m_filterRow )
			{
				shapeDef.enableCustomFiltering = true;
				shapeDef.material.customColor = b2_colorFuchsia;
			}
			else
			{
				shapeDef.enableCustomFiltering = false;
				shapeDef.material.customColor = 0;
			}

			float y = j * shift + yStart;
			for ( int i = 0; i < m_columnCount; ++i )
			{
				float x = i * shift - xCenter;
				b2Polygon box = b2MakeOffsetRoundedBox( 0.5f, 0.5f, { x, y }, b2Rot_identity, 0.1f );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
			}
		}

		m_maxBeginCount = 0;
		m_maxEndCount = 0;
		m_lastStepCount = 0;
	}

	void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			// stagger bodies to avoid bunching up events into a single update
			float yOffset = RandomFloatRange( -1.0f, 1.0f );
			bodyDef.position = { shift * i - xCenter, y + yOffset };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b2BodyId> zombies;

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = events.beginEvents + i;

			// shapes on begin touch are always valid

			ShapeUserData* userData = static_cast<ShapeUserData*>( b2Shape_GetUserData( event->sensorShapeId ) );

			if ( userData->active )
			{
				zombies.emplace( b2Shape_GetBody( event->visitorShapeId ) );
			}
			else
			{
				// Check custom filter correctness
				assert( userData->row != m_filterRow );

				// Modify color while overlapped with a sensor
				b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, &surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = events.endEvents + i;

			if ( b2Shape_IsValid( event->visitorShapeId ) == false )
			{
				continue;
			}

			// Restore color to default
			b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b2Shape_SetSurfaceMaterial( event->visitorShapeId, &surfaceMaterial );
		}

		for ( b2BodyId bodyId : zombies )
		{
			b2DestroyBody( bodyId );
		}

		int delay = 0x1F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 10.0f + m_rowCount * 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = b2MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = b2MaxInt( events.endCount, m_maxEndCount );
		DrawTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawTextLine( "max end touch events = %d", m_maxEndCount );
	}

	bool Filter( b2ShapeId idA, b2ShapeId idB )
	{
		ShapeUserData* userData = nullptr;
		if ( b2Shape_IsSensor( idA ) )
		{
			userData = (ShapeUserData*)b2Shape_GetUserData( idA );
		}
		else if ( b2Shape_IsSensor( idB ) )
		{
			userData = (ShapeUserData*)b2Shape_GetUserData( idB );
		}

		if ( userData != nullptr )
		{
			return userData->active == true || userData->row != m_filterRow;
		}

		return true;
	}

	static bool FilterFcn( b2ShapeId idA, b2ShapeId idB, void* context )
	{
		BenchmarkSensor* self = (BenchmarkSensor*)context;
		return self->Filter( idA, idB );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSensor( context );
	}

	static constexpr int m_columnCount = 40;
	static constexpr int m_rowCount = 40;
	int m_maxBeginCount;
	int m_maxEndCount;
	ShapeUserData m_passiveSensors[m_rowCount];
	ShapeUserData m_activeSensor;
	int m_lastStepCount;
	int m_filterRow;
};

static int benchmarkSensor = RegisterSample( "Benchmark", "Sensor", BenchmarkSensor::Create );

class BenchmarkCapacity : public Sample
{
public:
	explicit BenchmarkCapacity( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 150.0f };
			m_context->camera.zoom = 200.0f;
		}

		m_context->enableSleep = false;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position.y = -5.0f;
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 800.0f, 5.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_square = b2MakeSquare( 0.5f );
		m_done = false;
		m_reachCount = 0;
	}

	void Step() override
	{
		Sample::Step();

		float millisecondLimit = 20.0f;

		b2Profile profile = b2World_GetProfile( m_worldId );
		if ( profile.step > millisecondLimit )
		{
			m_reachCount += 1;
			if ( m_reachCount > 60 )
			{
				// Hit the millisecond limit 60 times in a row
				m_done = true;
			}
		}
		else
		{
			m_reachCount = 0;
		}

		if ( m_done == true )
		{
			return;
		}

		if ( ( m_stepCount & 0x1F ) != 0x1F )
		{
			return;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.y = 200.0f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		int count = 200;
		float x = -1.0f * count;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position.x = x;
			bodyDef.position.y += 0.5f;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &m_square );

			x += 2.0f;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCapacity( context );
	}

	b2Polygon m_square;
	int m_reachCount;
	bool m_done;
};

static int benchmarkCapacity = RegisterSample( "Benchmark", "Capacity", BenchmarkCapacity::Create );

class BlockBreakerVS final : public Sample
{
public:
	// ---------- Modèle de perso (lisible & rangé) ----------
	struct CharacterConfig
	{
		const char* id = "Unnamed";
		const char* category = "";

		struct General
		{
			b2HexColor color = b2_colorWhite;
			float spawnSpeedMul = 1.0f;

			bool useGlobalRadius = true;
			float radius = 0.80f;

			bool useGlobalRestitution = true;
			float restitution = 1.0f;

			bool useGlobalFriction = true;
			float friction = 0.0f;

			bool useGlobalGravityScale = true;
			float gravityScale = 1.0f;
		} general;

		struct Damage
		{
			enum Model
			{
				Unit,
				Fibonacci,
				Stacked
			} model = Unit;
			float stackPerHit = 0.0f;  // dmg += floor(stack)
			int instantBonusOnHit = 0; // +instant
		} damage;

		struct Speed
		{
			float addPerHit = 0.0f;
			float mulPerHit = 1.0f;
		} speed; // +m/s ou xMul
		struct Gravity
		{
			float addPerHit = 0.0f;
		} gravity; // gScale += ...
		struct Frict
		{
			float addPerHit = 0.0f;
		} friction; // friction += ...
		struct Grow
		{
			float addPerHit = 0.0f;
			float mulPerHit = 1.0f;
		} grow; // r += / r *=

		struct Clone
		{
			int perHit = 0;		// prioritaire
			int everyN = 0;		// fallback
			int maxPerBall = 0; // 0 = illimité
		} clone;

		struct Grenade
		{
			int base = 0;
			int incEveryN = 0;
			float radius = 0.20f;
			float speed = 7.0f;
			float ttl = 1.00f;
		} grenade;

		struct Drill
		{
			bool enabled = false;
			int perHit = 0;
			float size = 0.40f; // demi-hauteur
			float speed = 1.0f; // |vy| initiale vers le bas
			float ttl = 1.60f;

			float linearDampingBase = 0.0f;
			float linearDampingStep = 0.01f;
		} drill;
	};

	// ---------- Types ----------
	struct Arena
	{
		b2BodyId borderBody{ b2_nullBodyId };
		std::array<b2BodyId, 3> bars{ b2_nullBodyId, b2_nullBodyId, b2_nullBodyId };

		std::array<int, 3> lives{ 10, 100, 1000 };
		std::array<int, 3> maxLives{ 10, 100, 1000 };

		std::array<int8_t, 3> lastHitTeam{ -1, -1, -1 };
		std::array<uint8_t, 3> pendingKill{ 0, 0, 0 };

		std::array<b2ShapeId, 3> barShapes{ b2_nullShapeId, b2_nullShapeId, b2_nullShapeId };

		b2Vec2 center{ 0.0f, 0.0f };
		float innerW{ 15.0f };
		float innerH{ 12.0f };
		float wallT{ 0.10f };
	};

	struct BarUser
	{
		int arenaIndex = -1; // 0 = top, 1 = bottom
		int barIndex = -1;	 // 0..2
	};

	// UserData balle
	struct BallUser
	{
		int team = -1;
		const CharacterConfig* cfg = nullptr;

		// Runtime “post-hit”
		int hitCount = 0;
		float damageStack = 0.0f;
		int clonesMade = 0;
		bool isGrenade = false;
		bool isDrill = false;

		// NEW: seul le principal peut cloner
		bool isPrimary = true; // true pour les spawns initiaux, false pour les clones

		// HUD
		float speedScale = 1.0f;

		// Physique courante
		float baseRadius = 0.0f;
		float radius = 0.0f;
		float friction = 0.0f;
		float restitution = 1.0f; // restitution propre à la balle
		float gravityScale = 1.0f;
	};

	// catégories & masques (pas de collisions balles↔balles)
	static constexpr uint64_t CAT_WALL = 0x0001ull;
	static constexpr uint64_t CAT_BAR = 0x0002ull;
	static constexpr uint64_t CAT_BALL = 0x0004ull;

	static constexpr uint64_t MASK_WALL = CAT_BAR | CAT_BALL | CAT_WALL;
	static constexpr uint64_t MASK_BAR = CAT_BALL | CAT_WALL;
	static constexpr uint64_t MASK_BALL = CAT_BAR | CAT_WALL;

	// ---------- Helpers ----------
	static inline ImVec4 B2ToImVec4( b2HexColor c, float a = 1.0f )
	{
		const uint32_t u = (uint32_t)c;
		const float r = float( ( u >> 16 ) & 0xFF ) / 255.0f;
		const float g = float( ( u >> 8 ) & 0xFF ) / 255.0f;
		const float b = float( ( u ) & 0xFF ) / 255.0f;
		return ImVec4( r, g, b, a );
	}
	static inline ImU32 ImColFromHex( b2HexColor c, uint8_t a = 255 )
	{
		const uint8_t r = uint8_t( ( (uint32_t)c >> 16 ) & 0xFF );
		const uint8_t g = uint8_t( ( (uint32_t)c >> 8 ) & 0xFF );
		const uint8_t b = uint8_t( ( (uint32_t)c ) & 0xFF );
		return IM_COL32( r, g, b, a );
	}
	static inline uint8_t _clampU8( int v )
	{
		return (uint8_t)b2ClampInt( v, 0, 255 );
	}

	static inline b2HexColor LerpColor( b2HexColor a, b2HexColor b, float t )
	{
		t = b2ClampFloat( t, 0.0f, 1.0f );
		const int ar = ( (uint32_t)a >> 16 ) & 0xFF;
		const int ag = ( (uint32_t)a >> 8 ) & 0xFF;
		const int ab = ( (uint32_t)a ) & 0xFF;
		const int br = ( (uint32_t)b >> 16 ) & 0xFF;
		const int bg = ( (uint32_t)b >> 8 ) & 0xFF;
		const int bb = ( (uint32_t)b ) & 0xFF;
		const int rr = (int)std::lround( ar + ( br - ar ) * t );
		const int rg = (int)std::lround( ag + ( bg - ag ) * t );
		const int rb = (int)std::lround( ab + ( bb - ab ) * t );
		return (b2HexColor)( ( _clampU8( rr ) << 16 ) | ( _clampU8( rg ) << 8 ) | _clampU8( rb ) );
	}

	static inline void SetShapeColor( b2ShapeId s, b2HexColor c )
	{
		if ( B2_IS_NULL( s ) || !b2Shape_IsValid( s ) )
			return;
		b2SurfaceMaterial m = b2Shape_GetSurfaceMaterial( s );
		m.customColor = c;
		b2Shape_SetSurfaceMaterial( s, &m ); // API 3.1 attend un pointeur
	}

	// --- Utilitaire pour reconstruire un point de contact lors d’un BeginTouch ---
	static inline b2Vec2 ComputeBeginHitPoint( b2ShapeId barS, b2ShapeId ballS )
	{
		if ( B2_IS_NULL( barS ) || B2_IS_NULL( ballS ) )
			return b2Vec2{ 0.0f, 0.0f };

		b2BodyId ballB = b2Shape_GetBody( ballS );
		if ( B2_IS_NULL( ballB ) || !b2Body_IsValid( ballB ) )
			return b2Vec2{ 0.0f, 0.0f };

		const b2Vec2 cBall = b2Body_GetPosition( ballB );
		const b2Vec2 pOnBar = b2Shape_GetClosestPoint( barS, cBall );
		const b2Vec2 pOnBall = b2Shape_GetClosestPoint( ballS, pOnBar );
		return 0.5f * ( pOnBar + pOnBall ); // proxy monde propre
	}

	// ---------- Roster ----------
	static inline CharacterConfig MakeFibonacci()
	{
		CharacterConfig c;
		c.id = "Fibonacci";
		c.category = "damage";
		c.general.color = b2_colorLime;
		// Modèle de dégâts inchangé
		c.damage.model = CharacterConfig::Damage::Fibonacci;

		// >>> Gravité individuelle (au spawn)
		c.general.useGlobalGravityScale = false; // on ignore la gravité “globale”
		c.general.gravityScale = 0.5f;			 // <-- choisis ta valeur (1.0 = normal, >1 plus lourd, <1 plus flottant)

		return c;
	}
	static inline CharacterConfig MakeSlammy()
	{
		CharacterConfig c;
		c.id = "Slammy";
		c.category = "speed";
		c.general.color = b2_colorMediumVioletRed;
		c.general.spawnSpeedMul = 1.10f;
		c.speed.mulPerHit = 1.10f;
		return c;
	}
	static inline CharacterConfig MakeGravitron()
	{
		CharacterConfig c;
		c.id = "Gravitron";
		c.category = "gravity";
		c.general.color = b2_colorDeepSkyBlue;
		c.gravity.addPerHit = 0.15f;
		c.general.useGlobalRestitution = false;
		c.general.restitution = 0.99f;
		return c;
	}
	static inline CharacterConfig MakeMultiplier()
	{
		CharacterConfig c;
		c.id = "Multiplier";
		c.category = "clones";
		c.general.color = b2_colorGold;
		c.clone.perHit = 1;
		c.clone.maxPerBall = 0;
		return c;
	}
	static inline CharacterConfig MakeSpeedy()
	{
		CharacterConfig c;
		c.id = "Speedy";
		c.category = "speed";
		c.general.color = b2_colorSpringGreen;
		c.general.spawnSpeedMul = 1.10f;
		c.speed.addPerHit = 1.0f;
		return c;
	}
	static inline CharacterConfig MakeSticky()
	{
		CharacterConfig c;
		c.id = "Sticky";
		c.category = "friction";
		c.general.color = b2_colorTomato;
		c.friction.addPerHit = 0.05f;
		return c;
	}
	static inline CharacterConfig MakeSplodey()
	{
		CharacterConfig c;
		c.id = "Splodey";
		c.category = "damage";
		c.general.color = b2_colorOrangeRed;
		c.damage.model = CharacterConfig::Damage::Stacked;
		c.damage.stackPerHit = 1.0f;
		c.damage.instantBonusOnHit = 1;
		return c;
	}
	static inline CharacterConfig MakeCloudy()
	{
		CharacterConfig c;
		c.id = "Cloudy";
		c.category = "grenades";
		c.general.color = b2_colorSeaGreen;
		c.grenade.base = 1;
		c.grenade.incEveryN = 3;
		c.grenade.radius = 0.20f;
		c.grenade.speed = 7.0f;
		c.grenade.ttl = 1.00f;
		return c;
	}
	static inline CharacterConfig MakeGrower()
	{
		CharacterConfig c;
		c.id = "Grower";
		c.category = "grow";
		c.general.color = b2_colorOrchid;
		c.grow.addPerHit = 0.01f;
		return c;
	}
	static inline CharacterConfig MakeDrilley()
	{
		CharacterConfig c;
		c.id = "Drilley";
		c.category = "drilley";
		c.general.color = b2_colorBrown;
		c.drill.enabled = true;
		c.drill.perHit = 1;
		c.drill.size = 0.40f;
		c.drill.speed = 1.0f;
		c.drill.ttl = 1.60f;
		c.drill.linearDampingBase = 0.0f;
		c.drill.linearDampingStep = 0.01f;
		return c;
	}

	static inline const CharacterConfig kRoster[10] = { MakeFibonacci(), MakeSlammy(), MakeGravitron(), MakeMultiplier(),
														MakeSpeedy(),	 MakeSticky(), MakeSplodey(),	MakeCloudy(),
														MakeGrower(),	 MakeDrilley() };

	static inline const CharacterConfig* FindCharacterConfig( const std::string& name )
	{
		for ( const auto& c : kRoster )
			if ( name == c.id )
				return &c;
		return &kRoster[0];
	}

	// ---------- HUD utils ----------
	static inline std::string FormatThousand( int v )
	{
		std::string s = std::to_string( b2MaxInt( 0, v ) );
		std::string out;
		out.reserve( s.size() + s.size() / 3 );
		int n = (int)s.size();
		for ( int i = 0; i < n; ++i )
		{
			out.push_back( s[i] );
			int left = n - i - 1;
			if ( left > 0 && ( left % 3 ) == 0 )
				out.push_back( ',' );
		}
		return out;
	}
	static inline std::string ToK( int v )
	{
		return FormatThousand( v );
	}
	static inline std::string Fx( float f )
	{
		char buf[32];
		std::snprintf( buf, sizeof buf, "x%.2f", f );
		return buf;
	}
	static inline std::string F2( float f )
	{
		char buf[32];
		std::snprintf( buf, sizeof buf, "%.2f", f );
		return buf;
	}

	// ----- HUD edge labels -----
	float m_hudEdgeOffsetPx = 50.0f; // distance en pixels au bord extérieur des murs

	void DrawEdgeText( const Arena& a, bool top, const std::string& text )
	{
		const float yEdge = a.center.y + ( top ? ( +0.5f * a.innerH + 2.0f * a.wallT ) : ( -0.5f * a.innerH - 2.0f * a.wallT ) );
		b2Vec2 anchorWorld{ a.center.x, yEdge };
		b2Vec2 pScreen = ConvertWorldToScreen( &m_context->camera, anchorWorld );
		pScreen.y += top ? -m_hudEdgeOffsetPx : +m_hudEdgeOffsetPx;
		b2Vec2 pWorld = ConvertScreenToWorld( &m_context->camera, pScreen );
		DrawCenteredText( pWorld, text.c_str(), IM_COL32( 255, 255, 255, 255 ), true,
						  m_context->largeFont ? m_context->largeFont : ImGui::GetFont() );
	}

	// ---------- Ctor ----------
	explicit BlockBreakerVS( SampleContext* context )
		: Sample( context )
	{
		// Caméra
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 20.0f;
		}
		m_context->debugDraw.drawJoints = false;

		// presets : 1 balle chacun
		m_topName = "Fibonacci";
		m_bottomName = "Gravitron"; // montre la restitution per-character
		m_topCount = 1;
		m_botCount = 1;

		b2World_SetGravity( m_worldId, { 0.0f, -10.0f } );
		b2World_SetContactTuning( m_worldId, 240.0f, 1.0f, 1.0f );

		BuildArenas();
		SpawnParticipants();
	}

	~BlockBreakerVS() override
	{
		DestroyArena( m_arena[0] );
		DestroyArena( m_arena[1] );
		ClearBodies( m_topBalls );
		ClearBodies( m_botBalls );

		DestroyTimedBodies();
		m_drillContacts.clear();
		m_ballUsers.clear();
		m_hitFx.clear();
		m_pendingGrow.clear();
	}

	// ---------- GUI ----------
	void UpdateGui() override
	{
		// Toggle fenêtre avec TAB (hors saisie texte)
		if ( ImGui::IsKeyPressed( ImGuiKey_Tab ) && !ImGui::GetIO().WantTextInput )
			m_showGui = !m_showGui;

		if ( !m_showGui )
			return;

		ImGui::SetNextWindowSize( ImVec2( 460.0f, 520.0f ), ImGuiCond_Once );
		if ( ImGui::Begin( "BlockBreaker Pro" ) )
		{
			bool guiChanged = false;

			if ( ImGui::BeginTabBar( "bb_tabs" ) )
			{
				// ===== Participants =====
				if ( ImGui::BeginTabItem( "Participants" ) )
				{
					const CharacterConfig* topCfg = FindCharacterConfig( m_topName );
					const CharacterConfig* botCfg = FindCharacterConfig( m_bottomName );

					// --- TOP ---
					ImGui::PushStyleColor( ImGuiCol_Text, B2ToImVec4( topCfg->general.color ) );
					ImGui::TextUnformatted( "TOP" );
					ImGui::PopStyleColor();
					ImGui::SameLine();
					ImGui::ColorButton( "##topcol", B2ToImVec4( topCfg->general.color ),
										ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop, ImVec2( 16, 16 ) );

					if ( ImGui::BeginCombo( "Preset Top", m_topName.c_str() ) )
					{
						for ( const auto& cfg : kRoster )
						{
							const bool selected = ( m_topName == cfg.id );
							ImGui::PushStyleColor( ImGuiCol_Text, B2ToImVec4( cfg.general.color ) );
							if ( ImGui::Selectable( cfg.id, selected ) )
							{
								m_topName = cfg.id;
								guiChanged = true;
							}
							if ( selected )
								ImGui::SetItemDefaultFocus();
							ImGui::PopStyleColor();
						}
						ImGui::EndCombo();
					}

					{
						char buf[64];
						std::snprintf( buf, sizeof buf, "%s", m_topName.c_str() );
						if ( ImGui::InputText( "Nom Top", buf, sizeof buf ) )
						{
							if ( ImGui::IsItemDeactivatedAfterEdit() )
							{
								m_topName = buf;
								guiChanged = true;
							}
						}
					}

					if ( ImGui::SliderInt( "Nombre Top", &m_topCount, 0, 1000, "%d" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;

					ImGui::Separator();

					// --- BOTTOM ---
					ImGui::PushStyleColor( ImGuiCol_Text, B2ToImVec4( botCfg->general.color ) );
					ImGui::TextUnformatted( "BOTTOM" );
					ImGui::PopStyleColor();
					ImGui::SameLine();
					ImGui::ColorButton( "##botcol", B2ToImVec4( botCfg->general.color ),
										ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop, ImVec2( 16, 16 ) );

					if ( ImGui::BeginCombo( "Preset Bottom", m_bottomName.c_str() ) )
					{
						for ( const auto& cfg : kRoster )
						{
							const bool selected = ( m_bottomName == cfg.id );
							ImGui::PushStyleColor( ImGuiCol_Text, B2ToImVec4( cfg.general.color ) );
							if ( ImGui::Selectable( cfg.id, selected ) )
							{
								m_bottomName = cfg.id;
								guiChanged = true;
							}
							if ( selected )
								ImGui::SetItemDefaultFocus();
							ImGui::PopStyleColor();
						}
						ImGui::EndCombo();
					}

					{
						char buf[64];
						std::snprintf( buf, sizeof buf, "%s", m_bottomName.c_str() );
						if ( ImGui::InputText( "Nom Bottom", buf, sizeof buf ) )
						{
							if ( ImGui::IsItemDeactivatedAfterEdit() )
							{
								m_bottomName = buf;
								guiChanged = true;
							}
						}
					}

					if ( ImGui::SliderInt( "Nombre Bottom", &m_botCount, 0, 1000, "%d" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;

					ImGui::Separator();
					// ÉTALONS balles (globaux)
					if ( ImGui::SliderFloat( "Taille balle (rayon)", &m_ballRadius, 0.2f, 2.5f, "%.2f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Restitution (globale)", &m_ballRestitution, 0.0f, 1.5f, "%.2f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Friction (globale)", &m_ballFriction, 0.0f, 2.0f, "%.2f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;

					ImGui::Separator();
					if ( ImGui::Button( "Reset (participants + caméra)" ) )
					{
						guiChanged = true;
					}

					ImGui::EndTabItem();
				}

				// ===== Arènes =====
				if ( ImGui::BeginTabItem( "Arènes" ) )
				{
					if ( ImGui::SliderFloat( "Largeur interieure", &m_arenaW, 10.0f, 28.0f, "%.1f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Hauteur interieure", &m_arenaH, 10.0f, 28.0f, "%.1f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Epaisseur mur", &m_wallT, 0.1f, 1.0f, "%.2f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Ecart vertical", &m_verticalGap, 3.0f, 8.0f, "%.1f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "Jeu blocks-murs (monde)", &m_blockEdgeGap, 0.0f, 0.05f, "%.3f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;
					if ( ImGui::SliderFloat( "HUD distance (px)", &m_hudEdgeOffsetPx, 0.0f, 150.0f, "%.0f px" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;

					ImGui::SeparatorText( "FX" );
					ImGui::SliderFloat( "FX ring scale", &m_hitFxScale, 0.5f, 3.0f, "%.2fx" );

					if ( ImGui::Button( "Rebuild Arenas" ) )
						guiChanged = true;
					ImGui::EndTabItem();
				}

				// ===== Barres =====
				if ( ImGui::BeginTabItem( "Barres" ) )
				{
					if ( ImGui::SliderFloat( "Hauteur barre (half)", &m_barHalfH, 0.3f, 1.2f, "%.2f" ) )
						if ( ImGui::IsItemDeactivatedAfterEdit() )
							guiChanged = true;

					int tmp[3] = { m_blockLivesDefault[0], m_blockLivesDefault[1], m_blockLivesDefault[2] };
					if ( ImGui::InputInt3( "PV blocks (Haut->Milieu->Bas)", tmp ) )
					{
						if ( ImGui::IsItemDeactivatedAfterEdit() )
						{
							m_blockLivesDefault = { std::max( 1, tmp[0] ), std::max( 1, tmp[1] ), std::max( 1, tmp[2] ) };
							m_pendingRebuild = true; // -> Step() fera RebuildArenas() puis Respawn
						}
					}

					ImGui::TextUnformatted( "Arrondi désactivé (radius = 0)" );

					if ( ImGui::Checkbox( "Vies -- on hit event (sinon: begin)", &m_enableBarHitOnBegin ) )
						guiChanged = true;

					ImGui::EndTabItem();
				}

				ImGui::EndTabBar();
			}

			// RESET-ALL
			if ( guiChanged )
			{
				m_pendingRebuild = true;
				m_pendingRespawn = true;
				m_resetCameraNow = true;
			}
		}
		ImGui::End();
	}

	// ---------- Step ----------
	void Step() override
	{
		// 0) Auto-recovery
		if ( DetectAndRecoverWorldReset() )
		{
			DrawArenaDecor( m_arena[0], /*top=*/true );
			DrawArenaDecor( m_arena[1], /*top=*/false );
			DrawEdgeText( m_arena[0], /*top=*/true, MakeTeamMeterString( 0 ) );
			DrawEdgeText( m_arena[1], /*top=*/false, MakeTeamMeterString( 1 ) );
			DrawMidTitles();
			UpdateAndDrawHitFx( 1.0f / 60.0f );
			return;
		}

		// 1) Physique
		Sample::Step();

		// 1.b) TTL des projectiles temporaires (grenades/drills)
		UpdateTimedBodies();

		// 2) Événements
		b2ContactEvents contactEv = b2World_GetContactEvents( m_worldId );
		b2SensorEvents sensorEv = b2World_GetSensorEvents( m_worldId );

		// 2.a) HITS classiques
		if ( !m_enableBarHitOnBegin )
		{
			for ( int i = 0; i < contactEv.hitCount; ++i )
			{
				const b2ContactHitEvent& h = contactEv.hitEvents[i];
				if ( !b2Shape_IsValid( h.shapeIdA ) || !b2Shape_IsValid( h.shapeIdB ) )
					continue;

				const b2Filter fa = b2Shape_GetFilter( h.shapeIdA );
				const b2Filter fb = b2Shape_GetFilter( h.shapeIdB );
				const bool aBar = ( fa.categoryBits & CAT_BAR ) != 0;
				const bool bBar = ( fb.categoryBits & CAT_BAR ) != 0;
				const bool aBall = ( fa.categoryBits & CAT_BALL ) != 0;
				const bool bBall = ( fb.categoryBits & CAT_BALL ) != 0;
				if ( !( ( aBar && bBall ) || ( bBar && aBall ) ) )
					continue;

				const b2ShapeId barS = aBar ? h.shapeIdA : h.shapeIdB;
				const b2ShapeId ballS = aBall ? h.shapeIdA : h.shapeIdB;

				BallUser* bu = reinterpret_cast<BallUser*>( b2Shape_GetUserData( ballS ) );
				if ( !bu || bu->isDrill )
					continue;

				OnBarHit( barS, ballS, h.point ); // point monde exact
				DrawPoint( m_context->draw, h.point, 6.0f, b2_colorYellow );
			}
		}

		// 2.b) BEGIN de contact solide (option)
		if ( m_enableBarHitOnBegin )
		{
			for ( int i = 0; i < contactEv.beginCount; ++i )
			{
				const b2ContactBeginTouchEvent& e = contactEv.beginEvents[i];
				if ( B2_IS_NULL( e.shapeIdA ) || B2_IS_NULL( e.shapeIdB ) )
					continue;
				if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
					continue;

				const b2Filter fa = b2Shape_GetFilter( e.shapeIdA );
				const b2Filter fb = b2Shape_GetFilter( e.shapeIdB );
				const bool aBar = ( fa.categoryBits & CAT_BAR ) != 0;
				const bool bBar = ( fb.categoryBits & CAT_BAR ) != 0;
				const bool aBall = ( fa.categoryBits & CAT_BALL ) != 0;
				const bool bBall = ( fb.categoryBits & CAT_BALL ) != 0;
				if ( !( ( aBar && bBall ) || ( bBar && aBall ) ) )
					continue;

				const b2ShapeId barS = aBar ? e.shapeIdA : e.shapeIdB;
				const b2ShapeId ballS = aBall ? e.shapeIdA : e.shapeIdB;

				BallUser* bu = reinterpret_cast<BallUser*>( b2Shape_GetUserData( ballS ) );
				if ( !bu || bu->isDrill )
					continue;

				const b2Vec2 p = ComputeBeginHitPoint( barS, ballS );
				OnBarHit( barS, ballS, p );
			}
		}

		// 2.c) SENSOR EVENTS (drills)
		for ( int i = 0; i < sensorEv.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* ev = sensorEv.beginEvents + i;
			if ( !b2Shape_IsValid( ev->sensorShapeId ) || !b2Shape_IsValid( ev->visitorShapeId ) )
				continue;

			const b2Filter fv = b2Shape_GetFilter( ev->visitorShapeId );
			if ( ( fv.categoryBits & CAT_BAR ) == 0 )
				continue;

			BallUser* drillBU = reinterpret_cast<BallUser*>( b2Shape_GetUserData( ev->sensorShapeId ) );
			if ( !drillBU || drillBU->isDrill == false )
				continue;

			AddOrUpdateDrillContact( ev->visitorShapeId, ev->sensorShapeId, drillBU->team );
		}

		for ( int i = 0; i < sensorEv.endCount; ++i )
		{
			b2SensorEndTouchEvent* ev = sensorEv.endEvents + i;
			if ( !b2Shape_IsValid( ev->sensorShapeId ) || !b2Shape_IsValid( ev->visitorShapeId ) )
				continue;

			const b2Filter fv = b2Shape_GetFilter( ev->visitorShapeId );
			if ( ( fv.categoryBits & CAT_BAR ) == 0 )
				continue;

			RemoveDrillContact( ev->visitorShapeId, ev->sensorShapeId );
		}

		// 2.d) Tick DoT pour drills (1 PV / 0.1 s)
		UpdateDrillContactsDamage( 1.0f / 60.0f );

		// 2.e) Destructions différées
		ProcessPendingBarDestroy();

		// 2.f) Appliquer les GROW différés (anti double-hit)
		ApplyPendingGrowOps();

		// 3) Actions différées
		if ( m_pendingRebuild )
		{
			RebuildArenas();
			m_pendingRebuild = false;
			m_pendingRespawn = true;
		}
		if ( m_pendingRespawn )
		{
			RespawnParticipants();
			m_pendingRespawn = false;
			if ( m_resetCameraNow )
			{
				m_context->camera.center = { 0.0f, 0.0f };
				m_context->camera.zoom = 20.0f;
				m_resetCameraNow = false;
			}
		}

		// 4) Décor & HUD
		DrawArenaDecor( m_arena[0], /*top=*/true );
		DrawArenaDecor( m_arena[1], /*top=*/false );

		DrawEdgeText( m_arena[0], /*top=*/true, MakeTeamMeterString( 0 ) );
		DrawEdgeText( m_arena[1], /*top=*/false, MakeTeamMeterString( 1 ) );

		DrawMidTitles();

		// 5) FX (rings)
		UpdateAndDrawHitFx( 1.0f / 60.0f );
	}

	static Sample* Create( SampleContext* c )
	{
		return new BlockBreakerVS( c );
	}

private:
	// ====== Auto-recovery après reset monde ======
	static bool IsValidBody( b2BodyId id )
	{
		return B2_IS_NON_NULL( id ) && b2Body_IsValid( id );
	}

	bool DetectAndRecoverWorldReset()
	{
		bool need = false;
		if ( !IsValidBody( m_arena[0].borderBody ) || !IsValidBody( m_arena[1].borderBody ) )
			need = true;

		for ( int ai = 0; ai < 2 && !need; ++ai )
			for ( int bi = 0; bi < 3; ++bi )
				if ( B2_IS_NON_NULL( m_arena[ai].bars[bi] ) && !IsValidBody( m_arena[ai].bars[bi] ) )
					need = true;

		if ( !need )
			return false;

		for ( int ai = 0; ai < 2; ++ai )
		{
			m_arena[ai].borderBody = b2_nullBodyId;
			for ( int bi = 0; bi < 3; ++bi )
			{
				m_arena[ai].bars[bi] = b2_nullBodyId;
				m_arena[ai].barShapes[bi] = b2_nullShapeId;
				m_arena[ai].pendingKill[bi] = 0;
			}
		}

		m_topBalls.clear();
		m_botBalls.clear();
		m_ballUsers.clear();

		DestroyTimedBodies();
		m_drillContacts.clear();
		m_hitFx.clear();
		m_pendingGrow.clear();

		m_leadBall[0] = m_leadBall[1] = nullptr;
		ResetTeamStats();

		b2World_SetGravity( m_worldId, { 0.0f, -10.0f } );
		b2World_SetContactTuning( m_worldId, 240.0f, 0.01f, 1.0f );

		BuildArenas();
		SpawnParticipants();
		return true;
	}

	// ====== Arenas ======
	std::array<int, 3> m_blockLivesDefault{ 10, 100, 1000 };

	void BuildArenas()
	{
		const float midY = 0.0f;
		const float gap = m_verticalGap;
		BuildArena( m_arena[0], { 0.0f, midY + ( m_arenaH * 0.5f + gap ) } ); // TOP
		BuildArena( m_arena[1], { 0.0f, midY - ( m_arenaH * 0.5f + gap ) } ); // BOTTOM
	}

	void RebuildArenas()
	{
		DestroyArena( m_arena[0] );
		DestroyArena( m_arena[1] );
		BuildArenas();
	}

	void BuildArena( Arena& a, b2Vec2 center )
	{
		a.center = center;
		a.innerW = m_arenaW;
		a.innerH = m_arenaH;
		a.wallT = m_wallT;

		a.lives = m_blockLivesDefault;
		a.maxLives = m_blockLivesDefault;

		a.lastHitTeam = { -1, -1, -1 };
		a.pendingKill = { 0, 0, 0 };

		// 1) Murs — style BounceHouse : 4 segments "ligne"
		{
			if ( B2_IS_NON_NULL( a.borderBody ) && b2Body_IsValid( a.borderBody ) )
				b2DestroyBody( a.borderBody );
			a.borderBody = b2_nullBodyId;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_staticBody;
			bodyDef.position = center; // segments en coordonnées locales autour de 'center'
			a.borderBody = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef(); // pas de mat/filters ici (comme BounceHouse)

			const float hw = 0.5f * a.innerW;
			const float hh = 0.5f * a.innerH;

			{
				b2Segment s = { { -hw, -hh }, { +hw, -hh } };
				b2CreateSegmentShape( a.borderBody, &shapeDef, &s );
			} // bas
			{
				b2Segment s = { { +hw, -hh }, { +hw, +hh } };
				b2CreateSegmentShape( a.borderBody, &shapeDef, &s );
			} // droite
			{
				b2Segment s = { { +hw, +hh }, { -hw, +hh } };
				b2CreateSegmentShape( a.borderBody, &shapeDef, &s );
			} // haut
			{
				b2Segment s = { { -hw, +hh }, { -hw, -hh } };
				b2CreateSegmentShape( a.borderBody, &shapeDef, &s );
			} // gauche
		}


		// 2) Trois barres
		for ( int i = 0; i < 3; ++i )
		{
			const float hgap = 0.15f;
			const float halfH = m_barHalfH;
			const float fullH = 3.0f * ( 2.0f * halfH ) + 2.0f * hgap;
			const float y0 = +0.5f * fullH - halfH;
			const float yLocal = y0 - i * ( 2.0f * halfH + hgap );

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_staticBody;
			bd.position = center + b2Vec2{ 0.0f, yLocal };
			a.bars[i] = b2CreateBody( m_worldId, &bd );

			const float hw = 0.5f * a.innerW;
			const float halfWBar = b2MaxFloat( 0.0f, hw - m_blockEdgeGap );
			b2Polygon poly = b2MakeBox( halfWBar, halfH );
			poly.radius = 0.0f;

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.material = b2DefaultSurfaceMaterial();
			sd.material.friction = 0.0f;
			sd.material.restitution = 0.0f;
			sd.material.customColor = b2_colorWhite;
			sd.filter.categoryBits = CAT_BAR;
			sd.filter.maskBits = MASK_BAR;
			sd.enableContactEvents = true;
			sd.enableHitEvents = true;
			sd.enableSensorEvents = true;

			const int arenaIdx = ( &a == &m_arena[0] ) ? 0 : 1;
			m_barUsers[arenaIdx][i].arenaIndex = arenaIdx;
			m_barUsers[arenaIdx][i].barIndex = i;
			sd.userData = &m_barUsers[arenaIdx][i];

			a.barShapes[i] = b2CreatePolygonShape( a.bars[i], &sd, &poly );
		}
	}

	void DestroyArena( Arena& a )
	{
		for ( b2BodyId& b : a.bars )
		{
			if ( B2_IS_NON_NULL( b ) && b2Body_IsValid( b ) )
				b2DestroyBody( b );
			b = b2_nullBodyId;
		}
		if ( B2_IS_NON_NULL( a.borderBody ) && b2Body_IsValid( a.borderBody ) )
			b2DestroyBody( a.borderBody );
		a.borderBody = b2_nullBodyId;
	}

	// ====== Participants ======
	struct SpawnGlobals
	{
		float globalRadius, globalRestitution, globalFriction, globalGravityScale;
	};

	static inline void ApplySpawnTuning( const CharacterConfig& cfg, const SpawnGlobals& g,
										 /*in/out*/ b2BodyDef& bd,
										 /*in/out*/ b2ShapeDef& sd )
	{
		bd.linearVelocity = cfg.general.spawnSpeedMul * bd.linearVelocity;
		bd.gravityScale = cfg.general.useGlobalGravityScale ? g.globalGravityScale : cfg.general.gravityScale;

		sd.material.restitution = cfg.general.useGlobalRestitution ? g.globalRestitution : cfg.general.restitution;
		sd.material.friction = cfg.general.useGlobalFriction ? g.globalFriction : cfg.general.friction;
		sd.material.customColor = cfg.general.color;
	}

	static inline float InitialRadius( const CharacterConfig& cfg, const SpawnGlobals& g )
	{
		return cfg.general.useGlobalRadius ? g.globalRadius : cfg.general.radius;
	}

	BallUser* NewBallUser( int team, const CharacterConfig* cfg )
	{
		m_ballUsers.emplace_back();
		BallUser& bu = m_ballUsers.back();
		bu.team = team;
		bu.cfg = cfg;
		return &bu;
	}

	const CharacterConfig* CfgForTeam( int team ) const
	{
		return ( team == 0 ? m_topCfg : m_botCfg );
	}

	void SpawnParticipants()
	{
		m_topCfg = FindCharacterConfig( m_topName );
		m_botCfg = FindCharacterConfig( m_bottomName );

		ResetTeamStats();
		m_leadBall[0] = m_leadBall[1] = nullptr;

		SpawnGroup( m_arena[0], m_topBalls, m_topCount, *m_topCfg, /*team*/ 0, /*downward*/ true );
		SpawnGroup( m_arena[1], m_botBalls, m_botCount, *m_botCfg, /*team*/ 1, /*downward*/ false );

		InitDrillRuntimeFromDefs();
	}

	void RespawnParticipants()
	{
		b2World_SetGravity( m_worldId, { 0.0f, -10.0f } );
		b2World_SetContactTuning( m_worldId, 240.0f, 0.01f, 1.0f );

		ClearBodies( m_topBalls );
		ClearBodies( m_botBalls );
		DestroyTimedBodies();
		m_drillContacts.clear();
		m_ballUsers.clear();
		m_hitFx.clear();
		m_pendingGrow.clear();

		m_leadBall[0] = m_leadBall[1] = nullptr;
		ResetTeamStats();
		SpawnParticipants();
	}

	void ClearBodies( std::vector<b2BodyId>& list )
	{
		for ( b2BodyId id : list )
			if ( B2_IS_NON_NULL( id ) && b2Body_IsValid( id ) )
				b2DestroyBody( id );
		list.clear();
	}

	// Spawn group
	void SpawnGroup( const Arena& a, std::vector<b2BodyId>& out, int count, const CharacterConfig& cfg, int team, bool downward )
	{
		SpawnGlobals g{ m_ballRadius, m_ballRestitution, m_ballFriction, 1.0f };

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();

		const float r = InitialRadius( cfg, g );

		sd.density = 20.0f;
		sd.filter.categoryBits = CAT_BALL;
		sd.filter.maskBits = MASK_BALL;
		sd.enableContactEvents = true;
		sd.enableHitEvents = true;
		sd.enableSensorEvents = true;

		const float hw = 0.5f * a.innerW;
		const float hh = 0.5f * a.innerH;

		const float stepX = 2.2f * r;
		const float stepY = 2.2f * r;
		const float pad = std::max( 0.2f, r + 0.2f );
		const b2Vec2 anchor = a.center + b2Vec2{ -hw + pad, +hh - pad };

		for ( int i = 0; i < count; ++i )
		{
			const int gx = i % 16;
			const int gy = i / 16;
			const b2Vec2 p = anchor + b2Vec2{ gx * stepX, -gy * stepY };

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = p;
			bd.linearVelocity = downward ? b2Vec2{ +4.0f, -6.0f } : b2Vec2{ +4.0f, +6.0f };
			bd.allowFastRotation = true;
			bd.linearDamping = 0.0f;
			bd.angularDamping = 0.0f;

			ApplySpawnTuning( cfg, g, bd, sd );

			b2BodyId body = b2CreateBody( m_worldId, &bd );

			BallUser* bu = NewBallUser( team, &cfg );
			bu->baseRadius = r;
			bu->radius = r;
			bu->friction = sd.material.friction;
			bu->restitution = sd.material.restitution; // mémoriser la restitution effective
			bu->gravityScale = bd.gravityScale;

			b2Circle c{ { 0.0f, 0.0f }, r };
			sd.userData = bu;
			b2CreateCircleShape( body, &sd, &c );

			if ( m_leadBall[team] == nullptr )
				m_leadBall[team] = bu;

			out.push_back( body );
		}
	}

	// ===== Helpers de (re)création de shape =====
	void RecreateBallShape( b2BodyId body, BallUser& bu, float radius )
	{
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = b2ClampFloat( bu.restitution, 0.0f, 5.0f ); // restitution perso
		sd.material.friction = bu.friction;
		sd.material.customColor = GetTeamColor( bu.team );
		sd.density = 20.0f;
		sd.filter.categoryBits = CAT_BALL;
		sd.filter.maskBits = MASK_BALL;
		sd.enableContactEvents = true;
		sd.enableHitEvents = true;
		sd.enableSensorEvents = true;
		sd.userData = &bu;

		b2Circle c{ { 0.0f, 0.0f }, radius };
		b2CreateCircleShape( body, &sd, &c );
	}

	void UpdateBallFriction( b2ShapeId ballShape, BallUser& bu, float add )
	{
		const b2BodyId body = b2Shape_GetBody( ballShape );
		if ( !IsValidBody( body ) )
			return;

		bu.friction = ( bu.friction + add < 0.0f ) ? 0.0f : ( bu.friction + add );

		// recréation immédiate (ne change pas le rayon, donc pas de sur-hit)
		b2DestroyShape( ballShape, true );
		RecreateBallShape( body, bu, bu.radius );
	}

	// ====== Décor & textes ======
	b2HexColor GetTeamColor( int team ) const
	{
		if ( team == 0 )
			return ( m_topCfg ? m_topCfg->general.color : b2_colorLime );
		if ( team == 1 )
			return ( m_botCfg ? m_botCfg->general.color : b2_colorTomato );
		return b2_colorWhite;
	}

	void DrawArenaDecor( const Arena& a, bool isTop )
	{
		const b2HexColor teamColor = isTop ? ( m_topCfg ? m_topCfg->general.color : b2_colorLime )
										   : ( m_botCfg ? m_botCfg->general.color : b2_colorTomato );
		const b2Vec2 tl{ a.center.x - 0.5f * a.innerW + 0.8f, a.center.y + 0.5f * a.innerH - 0.8f };
		b2Transform xf = b2Transform_identity;
		xf.p = tl;
		DrawSolidCircle( m_context->draw, xf, 0.35f, teamColor );

		ImFont* blockFont = m_context->largeFont ? m_context->largeFont : ImGui::GetFont();

		for ( int i = 0; i < 3; ++i )
		{
			if ( !IsValidBody( a.bars[i] ) )
				continue;

			const b2Vec2 c = b2Body_GetPosition( a.bars[i] );
			const std::string txt = FormatThousand( a.lives[i] );
			DrawCenteredText( c, txt.c_str(), IM_COL32( 255, 255, 255, 255 ), true, blockFont );
		}
	}

	// ======= Helpers HUD Lead & modèles de dégâts =======
	static long long FibN_ll( int n )
	{
		if ( n <= 2 )
			return 1;
		long long a = 1, b = 1;
		for ( int i = 3; i <= n; ++i )
		{
			long long c = a + b;
			a = b;
			b = c;
			if ( b > 2000000000LL )
				return 2000000000LL;
		}
		return b;
	}

	int ComputeIncomingDamage( const BallUser& bu ) const
	{
		if ( bu.isGrenade || bu.isDrill )
			return 1;

		const CharacterConfig* C = bu.cfg;
		if ( !C )
			return 1;

		if ( C->damage.model == CharacterConfig::Damage::Fibonacci )
		{
			const int n = bu.hitCount + 1;
			return (int)FibN_ll( n );
		}

		const int stacked = (int)floorf( std::max( 0.0f, bu.damageStack ) );
		return 1 + stacked + C->damage.instantBonusOnHit;
	}

	int NextDamageForLead( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		if ( !lead )
			return 1;
		return ComputeIncomingDamage( *lead );
	}
	float LeadSpeedScale( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		return lead ? std::max( 0.0f, lead->speedScale ) : 1.0f;
	}
	float LeadGravityScale( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		return lead ? lead->gravityScale : 1.0f;
	}
	float LeadFriction( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		return lead ? std::max( 0.0f, lead->friction ) : 0.0f;
	}
	float LeadRadiusMul( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		if ( !lead )
			return 1.0f;
		return ( lead->baseRadius > 1e-6f ) ? ( lead->radius / lead->baseRadius ) : 1.0f;
	}

	int NextGrenadesForLead( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		if ( !lead || !lead->cfg )
			return 0;
		const auto& G = lead->cfg->grenade;
		if ( G.base <= 0 )
			return 0;
		int add = G.base;
		if ( G.incEveryN > 0 )
			add += ( lead->hitCount + 1 ) / G.incEveryN;
		return std::max( 0, add );
	}

	int NextClonesForLead( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		if ( !lead || !lead->cfg )
			return 0;
		if ( !lead->isPrimary )
			return 0;
		const auto& C = lead->cfg->clone;

		if ( C.perHit > 0 )
			return C.perHit;
		if ( C.everyN > 0 )
			return ( ( ( lead->hitCount + 1 ) % C.everyN ) == 0 ) ? 1 : 0;
		return 0;
	}

	std::string MakeTeamMeterString( int team ) const
	{
		const BallUser* lead = m_leadBall[team];
		if ( !lead || !lead->cfg )
			return "…";
		const CharacterConfig& C = *lead->cfg;

		if ( C.damage.model == CharacterConfig::Damage::Fibonacci || C.damage.model == CharacterConfig::Damage::Stacked ||
			 C.damage.instantBonusOnHit > 0 )
		{
			const int nd = NextDamageForLead( team );
			return std::string( "DMG " ) + ToK( nd );
		}

		if ( C.speed.addPerHit > 0.0f || C.speed.mulPerHit > 1.0f )
			return std::string( "SPD " ) + Fx( LeadSpeedScale( team ) );
		if ( C.gravity.addPerHit > 0.0f )
			return std::string( "GRV " ) + Fx( LeadGravityScale( team ) );
		if ( C.friction.addPerHit > 0.0f )
			return std::string( "FRC " ) + F2( LeadFriction( team ) );
		if ( C.grow.addPerHit > 0.0f || C.grow.mulPerHit > 1.0f )
			return std::string( "RAD " ) + Fx( LeadRadiusMul( team ) );
		if ( C.grenade.base > 0 )
			return std::string( "GRN +" ) + ToK( NextGrenadesForLead( team ) );
		if ( C.clone.perHit > 0 || C.clone.everyN > 0 )
			return std::string( "CLN +" ) + ToK( NextClonesForLead( team ) );
		if ( C.drill.enabled )
			return std::string( "DRL +" ) + ToK( b2MaxInt( 0, C.drill.perHit ) );

		return std::string( "HIT " ) + ToK( m_team[team].hits );
	}

	// Centre : juste le NOMBRE DE PERSOS + NOM
	void DrawMidTitles()
	{
		ImFont* big = m_context->largeFont ? m_context->largeFont : ImGui::GetFont();

		const std::string topTitle = FormatThousand( m_topCount ) + " " + m_topName;
		const std::string botTitle = FormatThousand( m_botCount ) + " " + m_bottomName;

		ImGui::PushFont( big );
		const ImVec2 szTop = ImGui::CalcTextSize( topTitle.c_str() );
		const ImVec2 szVS = ImGui::CalcTextSize( "VS" );
		const ImVec2 szBot = ImGui::CalcTextSize( botTitle.c_str() );
		ImGui::PopFont();

		const b2Vec2 wCenter = m_context->camera.center;
		const b2Vec2 sCenter = ConvertWorldToScreen( &m_context->camera, wCenter );
		const float yVS = sCenter.y;

		const float gap = -20.0f;

		const float yTop = yVS - ( 0.5f * szVS.y + gap + 0.5f * szTop.y );
		const float yBot = yVS + ( 0.5f * szVS.y + gap + 0.5f * szBot.y );

		const b2Vec2 wTop = ConvertScreenToWorld( &m_context->camera, b2Vec2{ sCenter.x, yTop } );
		const b2Vec2 wVs = ConvertScreenToWorld( &m_context->camera, b2Vec2{ sCenter.x, yVS } );
		const b2Vec2 wBot = ConvertScreenToWorld( &m_context->camera, b2Vec2{ sCenter.x, yBot } );

		const ImU32 topCol = ImColFromHex( m_topCfg ? m_topCfg->general.color : b2_colorLime );
		const ImU32 botCol = ImColFromHex( m_botCfg ? m_botCfg->general.color : b2_colorTomato );

		DrawCenteredText( wTop, topTitle.c_str(), topCol, true, big );
		DrawCenteredText( wVs, "VS", IM_COL32( 255, 255, 255, 255 ), false, big );
		DrawCenteredText( wBot, botTitle.c_str(), botCol, true, big );
	}

	void DrawCenteredText( b2Vec2 world, const char* txt, ImU32 col, bool outline, ImFont* font = nullptr )
	{
		ImDrawList* dl = ImGui::GetForegroundDrawList();
		ImFont* useFont = font ? font : ImGui::GetFont();

		b2Vec2 s = ConvertWorldToScreen( &m_context->camera, world );
		ImVec2 screen( s.x, s.y );

		ImGui::PushFont( useFont );
		ImVec2 size = ImGui::CalcTextSize( txt );
		ImGui::PopFont();

		const ImVec2 vp = ImGui::GetMainViewport()->Pos;
		ImVec2 p{ screen.x - size.x * 0.5f + vp.x, screen.y - size.y * 0.5f + vp.y };

		if ( outline )
		{
			for ( int dx = -2; dx <= 2; ++dx )
				for ( int dy = -2; dy <= 2; ++dy )
					if ( dx || dy )
						dl->AddText( useFont, useFont->FontSize, { p.x + (float)dx, p.y + (float)dy }, IM_COL32( 0, 0, 0, 200 ),
									 txt );
		}
		dl->AddText( useFont, useFont->FontSize, p, col, txt );
	}

	// ====== TIMED BODIES (grenades + drills) ======
	struct TimedBody
	{
		b2BodyId id{ b2_nullBodyId };
		float ttl{ 0.0f };
	};
	std::vector<TimedBody> m_timedBodies;

	void UpdateTimedBodies()
	{
		const float dt = 1.0f / 60.0f;
		for ( auto& tb : m_timedBodies )
			tb.ttl -= dt;

		for ( size_t i = 0; i < m_timedBodies.size(); )
		{
			if ( m_timedBodies[i].ttl <= 0.0f || !IsValidBody( m_timedBodies[i].id ) )
			{
				if ( IsValidBody( m_timedBodies[i].id ) )
					b2DestroyBody( m_timedBodies[i].id );
				m_timedBodies.erase( m_timedBodies.begin() + (ptrdiff_t)i );
			}
			else
			{
				++i;
			}
		}
	}

	void DestroyTimedBodies()
	{
		for ( auto& tb : m_timedBodies )
			if ( IsValidBody( tb.id ) )
				b2DestroyBody( tb.id );
		m_timedBodies.clear();
	}

	// ====== Logique de HIT & Effets ======
	void OnBarHit( b2ShapeId barShape, b2ShapeId ballShape, b2Vec2 hitPoint )
	{
		if ( B2_IS_NULL( barShape ) || B2_IS_NULL( ballShape ) )
			return;

		const BarUser* binfo = reinterpret_cast<const BarUser*>( b2Shape_GetUserData( barShape ) );
		if ( !binfo || binfo->arenaIndex < 0 || binfo->arenaIndex > 1 || binfo->barIndex < 0 || binfo->barIndex > 2 )
			return;

		BallUser* bu = reinterpret_cast<BallUser*>( b2Shape_GetUserData( ballShape ) );
		if ( !bu )
			return;

		// --- FX RING ---
		b2Vec2 fxPos = hitPoint;
		if ( fxPos.x == 0.0f && fxPos.y == 0.0f )
		{
			const b2BodyId b = b2Shape_GetBody( ballShape );
			if ( IsValidBody( b ) )
				fxPos = b2Body_GetPosition( b );
		}
		EmitHitFx( fxPos, GetTeamColor( bu->team ) );

		const int team = bu->team;
		const CharacterConfig* C = bu->cfg;

		const int damage = ComputeIncomingDamage( *bu );
		(void)ApplyDamageToBar( barShape, team, damage );

		if ( bu->isGrenade || bu->isDrill )
			return;

		m_leadBall[team] = bu;

		bu->hitCount++;
		m_team[team].hits += 1;

		if ( C )
			bu->damageStack += C->damage.stackPerHit;

		// === Speed ===
		if ( C && ( C->speed.addPerHit > 0.0f || C->speed.mulPerHit > 1.0f ) )
		{
			const b2BodyId body = b2Shape_GetBody( ballShape );
			if ( IsValidBody( body ) )
			{
				b2Vec2 v = b2Body_GetLinearVelocity( body );
				float spd = b2Length( v );
				if ( spd < 1e-5f )
				{
					v = { 1.0f, 0.0f };
					spd = 1.0f;
				}
				const b2Vec2 dir = ( 1.0f / spd ) * v;

				float newSpd = spd;
				if ( C->speed.addPerHit > 0.0f )
					newSpd = spd + C->speed.addPerHit;
				else
					newSpd = spd * C->speed.mulPerHit;

				b2Body_SetLinearVelocity( body, dir * newSpd );

				const float factor = ( spd > 1e-5f ) ? ( newSpd / spd ) : 1.0f;
				bu->speedScale *= factor;
				m_team[team].maxSpeedScale = b2MaxFloat( m_team[team].maxSpeedScale, bu->speedScale );
			}
		}

		// === Grow === (désormais DIFFÉRÉ pour éviter le double-hit)
		if ( C && ( C->grow.addPerHit > 0.0f || C->grow.mulPerHit > 1.0f ) )
		{
			ScheduleGrowBall( ballShape, *bu, *C );
		}

		// === Sticky / Friction ===
		if ( C && C->friction.addPerHit > 0.0f )
		{
			UpdateBallFriction( ballShape, *bu, C->friction.addPerHit );
		}

		// === Gravity ===
		if ( C && C->gravity.addPerHit > 0.0f )
		{
			const b2BodyId body = b2Shape_GetBody( ballShape );
			if ( IsValidBody( body ) )
			{
				const float g0 = b2Body_GetGravityScale( body );
				const float g1 = g0 + C->gravity.addPerHit;
				b2Body_SetGravityScale( body, g1 );
				bu->gravityScale = g1;
				m_team[team].maxGravityScale = b2MaxFloat( m_team[team].maxGravityScale, g1 );
			}
		}

		// === Multiplier / Clones ===
		if ( C && ( C->clone.perHit > 0 || C->clone.everyN > 0 ) && bu->isPrimary &&
			 ( C->clone.maxPerBall == 0 || bu->clonesMade < C->clone.maxPerBall ) )
		{
			int spawnN = 0;

			if ( C->clone.perHit > 0 )
			{
				spawnN = ( C->clone.maxPerBall == 0 ) ? C->clone.perHit
													  : b2MinInt( C->clone.perHit, C->clone.maxPerBall - bu->clonesMade );
			}
			else if ( C->clone.everyN > 0 && ( bu->hitCount % C->clone.everyN ) == 0 )
			{
				spawnN = ( C->clone.maxPerBall == 0 || bu->clonesMade < C->clone.maxPerBall ) ? 1 : 0;
			}

			if ( spawnN > 0 )
			{
				const b2BodyId src = b2Shape_GetBody( ballShape );
				if ( IsValidBody( src ) )
				{
					for ( int i = 0; i < spawnN; ++i )
						SpawnCloneFrom( src, *bu );
					bu->clonesMade += spawnN;
				}
			}
		}

		// === Cloudy / Grenades ===
		if ( C && C->grenade.base > 0 )
		{
			int add = C->grenade.base;
			if ( C->grenade.incEveryN > 0 )
				add += bu->hitCount / C->grenade.incEveryN;

			const b2BodyId src = b2Shape_GetBody( ballShape );
			b2Vec2 origin = IsValidBody( src ) ? b2Body_GetPosition( src ) : hitPoint;
			SpawnGrenadeBurst( origin, team, C->grenade, add );
		}

		// === Drilley / Drills ===
		if ( C && C->drill.enabled && C->drill.perHit > 0 )
		{
			const b2BodyId src = b2Shape_GetBody( ballShape );
			const b2Vec2 origin = IsValidBody( src ) ? b2Body_GetPosition( src ) : hitPoint;

			for ( int i = 0; i < C->drill.perHit; ++i )
			{
				const int idx = m_drillSpawnCount[team]++;
				const float damping = b2MaxFloat( 0.0f, C->drill.linearDampingBase + idx * C->drill.linearDampingStep );
				SpawnDrill( origin, b2Vec2{ 0.0f, -b2MaxFloat( 0.0f, C->drill.speed ) }, team, C->drill, damping );
			}
		}
	}

	// Retourne le dégât réellement appliqué + recolore le block via customColor
	int ApplyDamageToBar( b2ShapeId barShape, int team, int amount )
	{
		if ( amount <= 0 )
			return 0;
		const BarUser* info = reinterpret_cast<const BarUser*>( b2Shape_GetUserData( barShape ) );
		if ( !info )
			return 0;

		Arena& A = m_arena[info->arenaIndex];
		const int bi = info->barIndex;

		if ( A.pendingKill[bi] || !IsValidBody( A.bars[bi] ) )
			return 0;

		int& v = A.lives[bi];

		int applied = 0;
		if ( v > 0 )
		{
			applied = b2MinInt( amount, v );
			v -= applied;
		}

		if ( team >= 0 )
		{
			A.lastHitTeam[bi] = (int8_t)team;
			m_team[team].damage += applied;
		}

		const int vmax = b2MaxInt( 1, A.maxLives[bi] );
		const float lifeRatio = static_cast<float>( b2ClampInt( A.lives[bi], 0, vmax ) ) / static_cast<float>( vmax );
		const float lost = 1.0f - lifeRatio;
		const b2HexColor tcol = GetTeamColor( A.lastHitTeam[bi] );
		const b2HexColor mixed = LerpColor( b2_colorWhite, tcol, b2ClampFloat( lost, 0.0f, 1.0f ) );
		if ( b2Shape_IsValid( A.barShapes[bi] ) )
			SetShapeColor( A.barShapes[bi], mixed );

		if ( v <= 0 )
		{
			A.lives[bi] = 0;
			A.pendingKill[bi] = 1;
		}

		return applied;
	}

	// ====== Spawns secondaires ======
	void SpawnCloneFrom( b2BodyId src, const BallUser& srcBU )
	{
		if ( !IsValidBody( src ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = b2Body_GetPosition( src ) + b2Vec2{ RandomFloatRange( -0.15f, 0.15f ), RandomFloatRange( -0.15f, 0.15f ) };
		bd.linearVelocity =
			b2Body_GetLinearVelocity( src ) + b2Vec2{ RandomFloatRange( -1.0f, 1.0f ), RandomFloatRange( -1.0f, 1.0f ) };
		bd.allowFastRotation = true;
		bd.linearDamping = 0.0f;
		bd.angularDamping = 0.0f;
		bd.gravityScale = srcBU.gravityScale;

		b2BodyId body = b2CreateBody( m_worldId, &bd );

		BallUser* bu = NewBallUser( srcBU.team, srcBU.cfg );
		bu->isPrimary = false; // <-- clone : ne peut pas re-cloner
		bu->baseRadius = srcBU.baseRadius;
		bu->radius = srcBU.radius;
		bu->speedScale = srcBU.speedScale;
		bu->friction = srcBU.friction;
		bu->restitution = srcBU.restitution; // <-- héritage restitution
		bu->gravityScale = srcBU.gravityScale;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = b2ClampFloat( bu->restitution, 0.0f, 5.0f );
		sd.material.friction = bu->friction;
		sd.material.customColor = GetTeamColor( srcBU.team );
		sd.density = 20.0f;
		sd.filter.categoryBits = CAT_BALL;
		sd.filter.maskBits = MASK_BALL;
		sd.enableContactEvents = true;
		sd.enableHitEvents = true;
		sd.enableSensorEvents = true;
		sd.userData = bu;

		b2Circle c{ { 0.0f, 0.0f }, bu->radius };
		b2CreateCircleShape( body, &sd, &c );

		m_team[srcBU.team].clones += 1;

		if ( srcBU.team == 0 )
			m_topBalls.push_back( body );
		else
			m_botBalls.push_back( body );
	}

	void SpawnGrenadeBurst( const b2Vec2& origin, int team, const CharacterConfig::Grenade& G, int count )
	{
		const float twoPi = 6.28318530718f;
		for ( int i = 0; i < count; ++i )
		{
			const float a = twoPi * ( (float)i / (float)count ) + RandomFloatRange( -0.35f, 0.35f );
			const b2Vec2 dir{ cosf( a ), sinf( a ) };
			SpawnGrenade( origin, dir * G.speed, team, G );
		}
		m_team[team].grenades += b2MaxInt( 0, count );
	}

	void SpawnGrenade( b2Vec2 pos, b2Vec2 vel, int team, const CharacterConfig::Grenade& G )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = vel;
		bd.gravityScale = 1.0f;
		bd.allowFastRotation = true;
		bd.linearDamping = 0.0f;
		bd.angularDamping = 0.0f;

		b2BodyId body = b2CreateBody( m_worldId, &bd );

		BallUser* bu = NewBallUser( team, nullptr );
		bu->isGrenade = true;
		bu->baseRadius = G.radius;
		bu->radius = G.radius;
		bu->friction = 0.6f;
		bu->restitution = 0.6f;
		bu->gravityScale = 1.0f;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = 0.6f;
		sd.material.friction = 0.0f;
		sd.material.customColor = GetTeamColor( team );
		sd.density = 20.0f;
		sd.filter.categoryBits = CAT_BALL;
		sd.filter.maskBits = MASK_BALL;
		sd.enableContactEvents = true;
		sd.enableHitEvents = true;
		sd.enableSensorEvents = true;
		sd.userData = bu;

		b2Circle c{ { 0.0f, 0.0f }, G.radius };
		b2CreateCircleShape( body, &sd, &c );

		m_timedBodies.push_back( TimedBody{ body, G.ttl } );
	}

	// ====== Drilley: drills triangles sensors ======
	void SpawnDrill( b2Vec2 pos, b2Vec2 vel, int team, const CharacterConfig::Drill& D, float linearDamping )
	{
		const float h = D.size;
		const float halfBase = ( 2.0f * h ) / sqrtf( 3.0f );

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = vel;
		bd.gravityScale = 0.25f;
		bd.allowFastRotation = false;
		bd.linearDamping = b2MaxFloat( 0.0f, linearDamping );
		bd.angularDamping = 0.0f;

		b2BodyId body = b2CreateBody( m_worldId, &bd );

		BallUser* bu = NewBallUser( team, nullptr );
		bu->isDrill = true;
		bu->baseRadius = h;
		bu->radius = h;
		bu->friction = 0.0f;
		bu->restitution = 0.0f;
		bu->gravityScale = bd.gravityScale;

		b2Vec2 pts[3] = { { -halfBase, +h }, { +halfBase, +h }, { 0.0f, -h } };
		b2Hull hull = b2ComputeHull( pts, 3 );
		b2Polygon poly = b2MakePolygon( &hull, 0.0f );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = 0.0f;
		sd.material.restitution = 0.0f;
		sd.material.customColor = b2_colorBrown;
		sd.density = 0.25f;
		sd.filter.categoryBits = CAT_BALL;
		sd.filter.maskBits = MASK_BALL;
		sd.isSensor = true;
		sd.enableContactEvents = true; // begin/end
		sd.enableHitEvents = false;
		sd.enableSensorEvents = true;
		sd.userData = bu;

		b2CreatePolygonShape( body, &sd, &poly );

		m_timedBodies.push_back( TimedBody{ body, b2MaxFloat( 0.05f, D.ttl ) } );
	}

	// --- Drills: contacts persistants (sensor) ---
	struct DrillContact
	{
		b2ShapeId barShape{ b2_nullShapeId };
		b2ShapeId drillShape{ b2_nullShapeId };
		int team{ -1 };
		float acc{ 0.0f };
	};
	std::vector<DrillContact> m_drillContacts;

	void AddOrUpdateDrillContact( b2ShapeId barS, b2ShapeId drillS, int team )
	{
		for ( auto& dc : m_drillContacts )
		{
			if ( dc.barShape.index1 == barS.index1 && dc.drillShape.index1 == drillS.index1 )
				return;
		}
		m_drillContacts.push_back( DrillContact{ barS, drillS, team, 0.0f } );
	}

	void RemoveDrillContact( b2ShapeId barS, b2ShapeId drillS )
	{
		for ( size_t i = 0; i < m_drillContacts.size(); ++i )
		{
			auto& dc = m_drillContacts[i];
			if ( dc.barShape.index1 == barS.index1 && dc.drillShape.index1 == drillS.index1 )
			{
				m_drillContacts.erase( m_drillContacts.begin() + (ptrdiff_t)i );
				return;
			}
		}
	}

	void CleanupInvalidDrillContacts()
	{
		for ( size_t i = 0; i < m_drillContacts.size(); )
		{
			auto& dc = m_drillContacts[i];
			const bool ok = b2Shape_IsValid( dc.barShape ) && b2Shape_IsValid( dc.drillShape );
			if ( !ok )
				m_drillContacts.erase( m_drillContacts.begin() + (ptrdiff_t)i );
			else
				++i;
		}
	}

	void UpdateDrillContactsDamage( float dt )
	{
		const float tick = 0.10f; // 1 PV / 0.1 s
		for ( auto& dc : m_drillContacts )
		{
			if ( !b2Shape_IsValid( dc.barShape ) || !b2Shape_IsValid( dc.drillShape ) )
				continue;

			dc.acc += dt;
			while ( dc.acc >= tick )
			{
				dc.acc -= tick;
				const int applied = ApplyDamageToBar( dc.barShape, dc.team, 1 );

				// FX visuel au niveau du drill (fallback: centre de la barre)
				b2Vec2 posFx{ 0.0f, 0.0f };
				b2BodyId dBody = b2Shape_GetBody( dc.drillShape );
				if ( IsValidBody( dBody ) )
					posFx = b2Body_GetPosition( dBody );
				else
				{
					b2BodyId barBody = b2Shape_GetBody( dc.barShape );
					if ( IsValidBody( barBody ) )
						posFx = b2Body_GetPosition( barBody );
				}

				if ( applied > 0 )
					EmitHitFx( posFx, GetTeamColor( dc.team ), /*life*/ 0.30f, /*r0*/ 4.0f, /*r1*/ 28.0f, /*thickness*/ 3.0f );
			}
		}
	}

	// ====== Grow différé (anti double-hit) ======
	struct PendingGrowOp
	{
		b2ShapeId shape; // shape à détruire en fin de step
		b2BodyId body;	 // corps de la balle
		BallUser* bu;	 // pointeur stable (liste)
		float newRadius; // nouveau rayon à appliquer
	};
	std::vector<PendingGrowOp> m_pendingGrow;

	void ScheduleGrowBall( b2ShapeId ballShape, BallUser& bu, const CharacterConfig& C )
	{
		const b2BodyId body = b2Shape_GetBody( ballShape );
		if ( !IsValidBody( body ) )
			return;

		float target = bu.radius;
		if ( C.grow.addPerHit > 0.0f )
			target = bu.radius + C.grow.addPerHit;
		else
			target = bu.radius * C.grow.mulPerHit;

		const float newR = target;
		const float visMul = newR / b2MaxFloat( 1e-6f, bu.baseRadius );
		m_team[bu.team].maxRadiusMul = b2MaxFloat( m_team[bu.team].maxRadiusMul, visMul );

		if ( newR <= bu.radius * 1.000001f )
			return;

		// Fusionner si une op existe déjà pour ce body (prendre le plus grand rayon)
		for ( auto& op : m_pendingGrow )
		{
			if ( op.body.index1 == body.index1 )
			{
				op.newRadius = b2MaxFloat( op.newRadius, newR );
				return;
			}
		}

		m_pendingGrow.push_back( PendingGrowOp{ ballShape, body, &bu, newR } );
	}

	void ApplyPendingGrowOps()
	{
		for ( auto& op : m_pendingGrow )
		{
			if ( !IsValidBody( op.body ) )
				continue;
			if ( b2Shape_IsValid( op.shape ) )
				b2DestroyShape( op.shape, true ); // détruire l'ancienne shape en fin de step

			// recréer avec restitution propre au perso
			RecreateBallShape( op.body, *op.bu, op.newRadius );
			op.bu->radius = op.newRadius;
		}
		m_pendingGrow.clear();
	}

	// ====== Hit → vies & destroy ======
	void ProcessPendingBarDestroy()
	{
		for ( int ai = 0; ai < 2; ++ai )
		{
			Arena& A = m_arena[ai];
			for ( int bi = 0; bi < 3; ++bi )
			{
				if ( A.pendingKill[bi] && IsValidBody( A.bars[bi] ) )
				{
					b2DestroyBody( A.bars[bi] );
					A.bars[bi] = b2_nullBodyId;
					A.barShapes[bi] = b2_nullShapeId;
					A.pendingKill[bi] = 0;
				}
			}
		}

		CleanupInvalidDrillContacts();
	}

	// ====== Utils ======
	static float RandomFloatRange( float a, float b )
	{
		float t = (float)rand() / (float)RAND_MAX;
		return a + ( b - a ) * t;
	}

	// ====== Hit FX (rings) ======
	struct HitFx
	{
		b2Vec2 wpos;			// position monde
		b2HexColor base;		// couleur
		float age = 0.0f;		// temps écoulé
		float life = 0.35f;		// durée totale
		float r0 = 3.0f;		// rayon initial (px écran)
		float r1 = 22.0f;		// rayon final (px écran)
		float thickness = 2.0f; // épaisseur trait
	};
	std::vector<HitFx> m_hitFx;

	static inline float _smoothstep01( float t )
	{
		t = b2ClampFloat( t, 0.0f, 1.0f );
		return t * t * ( 3.0f - 2.0f * t );
	}

	void EmitHitFx( b2Vec2 worldPos, b2HexColor col, float life = 0.40f, float r0 = 6.0f, float r1 = 40.0f,
					float thickness = 3.0f )
	{
		m_hitFx.push_back( HitFx{ worldPos, col, 0.0f, b2MaxFloat( 0.06f, life ), r0, r1, thickness } );
	}

	void UpdateAndDrawHitFx( float dt )
	{
		ImDrawList* dl = ImGui::GetForegroundDrawList();
		const ImVec2 vp = ImGui::GetMainViewport()->Pos;

		for ( size_t i = 0; i < m_hitFx.size(); )
		{
			HitFx& fx = m_hitFx[i];
			fx.age += dt;
			const float t = b2ClampFloat( fx.age / fx.life, 0.0f, 1.0f );
			const float e = _smoothstep01( t );
			const float a = ( 1.0f - t ) * ( 1.0f - t );

			b2Vec2 sp = ConvertWorldToScreen( &m_context->camera, fx.wpos );

			const float radPx = ( fx.r0 + ( fx.r1 - fx.r0 ) * e ) * m_hitFxScale;
			const float thick = fx.thickness * m_hitFxScale;

			const ImU32 col = ImColFromHex( fx.base, (uint8_t)std::lround( 255.0f * b2ClampFloat( a, 0.0f, 1.0f ) ) );
			dl->AddCircle( ImVec2( sp.x + vp.x, sp.y + vp.y ), radPx, col, 32, thick );

			if ( fx.age >= fx.life )
				m_hitFx.erase( m_hitFx.begin() + (ptrdiff_t)i );
			else
				++i;
		}
	}

private:
	// arènes + users stables pour userData
	Arena m_arena[2];
	BarUser m_barUsers[2][3]{};

	// groupes
	std::vector<b2BodyId> m_topBalls;
	std::vector<b2BodyId> m_botBalls;

	// userData balles
	std::list<BallUser> m_ballUsers;

	// définitions de perso actives
	const CharacterConfig* m_topCfg = nullptr;
	const CharacterConfig* m_botCfg = nullptr;

	// titres / noms
	std::string m_topName, m_bottomName;

	// NB balles
	int m_topCount = 1;
	int m_botCount = 1;

	// géométrie / layout
	float m_arenaW = 15.0f, m_arenaH = 12.0f, m_wallT = 0.1f, m_verticalGap = 3.0f;

	// barres (blocks)
	float m_barHalfH = 1.0f;
	bool m_enableBarHitOnBegin = true;

	// ajustement horizontal des blocks (pile entre les murs)
	float m_blockEdgeGap = 0.05f;

	// balles (étalons sliders globaux)
	float m_ballRadius = 0.80f;
	float m_ballRestitution = 1.0f;
	float m_ballFriction = 0.0f;

	// flags différés
	bool m_pendingRebuild = false;
	bool m_pendingRespawn = false;
	bool m_resetCameraNow = false;

	// GUI
	bool m_showGui = true;

	float m_hitFxScale = 1.5f;

	// ====== Stats d'équipe pour HUD ======
	struct TeamStats
	{
		int damage = 0;
		int hits = 0;
		int clones = 0;
		int grenades = 0;
		float maxSpeedScale = 1.0f;
		float maxRadiusMul = 1.0f;
		float maxGravityScale = 1.0f;
	};
	TeamStats m_team[2];

	// --- Runtime Drilley (par équipe) ---
	int m_drillSpawnCount[2] = { 0, 0 };

	void InitDrillRuntimeFromDefs()
	{
		m_drillSpawnCount[0] = 0;
		m_drillSpawnCount[1] = 0;
	}

	// ====== Lead ball par équipe ======
	BallUser* m_leadBall[2] = { nullptr, nullptr }; // [0]=TOP, [1]=BOTTOM

	void ResetTeamStats()
	{
		m_team[0] = TeamStats{};
		m_team[1] = TeamStats{};
	}
};

static int sampleBlockBreakerVS = RegisterSample( "VS Games", "BlockBreaker VS", BlockBreakerVS::Create );

class WeaponsBallsVS final : public Sample
{
public:
	// ========= Types & constantes =========

	struct ArenaConfig
	{
		float halfWidth = 10.0f;
		float halfHeight = 10.0f;
		float thickness = 0.5f;
		float gap = 0.0f;
		float inset = 0.0f;
		int bottomSegments = 1;
		int topSegments = 1;
		int leftSegments = 1;
		int rightSegments = 1;

		uint32_t color = RGB( 255, 255, 255 );
	};

	// --- Death Poof moving circles -------------------------------------------------
	struct DeathPoofBall
	{
		b2Vec2 p;		  // position monde
		b2Vec2 v;		  // vitesse monde (m/s)
		float r;		  // rayon (monde)
		float damping;	  // amortissement linéaire (s^-1) appliqué à v
		uint32_t fillRGB; // couleur de remplissage (couleur joueur)
		uint32_t edgeRGB; // couleur du bord (un peu plus foncé)
		double t0, t1;	  // début / fin (disparition)
		double last;	  // dernier timestamp update
	};

	std::vector<DeathPoofBall> m_deathPoofBalls;

	// --- Death Poof rings ---------------------------------------------------------

	struct UnarmedTuning
	{
		// Vitesse minimale dynamique
		float minVel = 15.0f;
		float minVelCap = 100.0f;
		float minVelGrow = 1.0f;

		// Ghost trail
		float ghostRadius = kCharacterRadius + kUnarmedRingPad;
		float ghostMinDist = 0.1f;
		double ghostTTL = 1.0;
		int ghostMax = 10;
	};

	struct SlashLine
	{
		b2Vec2 A, B;
		double start;
		double dur;
	};
	struct ArrowBurst
	{
		int total = 1;
		int shot = 0;
		double nextTime = 0;
		bool active = false;
	};
	struct GhostCircle
	{
		b2Vec2 p;
		double death;
	};

	struct BoomerangState
	{
		b2BodyId body = b2_nullBodyId; // body de l’arme (pour le test de freeze)
		double phase = 0.0;
		double lastTime = 0.0;

		float reachA = 1.0f;
		float reachB = 1.0f;
		float animSpeed = 1.0f;

		int hitCount = 0;
	};

	// key = b2StoreBodyId(boomerangBody)
	std::unordered_map<uint64_t, BoomerangState> m_boomerangs;

	struct PairInvertState
	{
		bool canSwitch = true;
		bool touching = false;
		double lastBegin = 0.0;
		double lastSwitch = -1.0;
	};

	struct MotorBlend
	{
		b2JointId j = b2_nullJointId;
		float v0 = 0.f;
		float v1 = 0.f;
		double t0 = 0.0;
		double t1 = 0.0;
	};
	struct AudioBanks
	{
		int rebound = -1;
		int explosion_blast = -1;
		int hits = -1;
		int melee_hit = -1;
		int motor_invert = -1;

		// Projectiles
		int proj_impact = -1;

		int proj_bow = -1;
		int proj_crossbow = -1;
		int proj_vampire = -1;
		int proj_shuriken = -1;
		int proj_frost = -1;
		int proj_explosion = -1;
		int proj_electric_staff = -1;
		int proj_poison = -1;
		int proj_turret = -1;
		int proj_flask = -1;

		// Melee / weapons
		int w_bow = -1;
		int w_crossbow = -1;
		int w_sword = -1;
		int w_axe = -1;
		int w_dagger = -1;
		int w_katana = -1;
		int w_trident = -1;
		int w_hammer = -1;
		int w_poison_blowgun = -1;
		int w_club = -1;
		int w_scythe = -1;
		int w_spear = -1;
		int w_kunai = -1;
		int w_boomerang = -1;
		int w_shuriken = -1;
		int w_big_sword = -1;
		int w_electric_staff = -1;
		int w_explosion_staff = -1;
		int w_frost_staff = -1;
		int w_mahoraga = -1;
		int w_vampire_knife = -1;
		int w_shield = -1;
		int w_wrench = -1;
		int w_unarmed = -1;
		int w_lance = -1;

		// --- Nouveaux (pixel_art) ---
		int w_glaive = -1;
		int w_sickle_r = -1;
		int w_flask = -1;

	} m_banks;

	struct FreezeData
	{
		b2BodyId body = b2_nullBodyId;
		b2JointId joint = b2_nullJointId;

		b2Vec2 savedLinearVelocity{ 0.f, 0.f };
		float savedAngularVelocity = 0.f;

		bool hadMotor = false;
		bool motorWasEnabled = false;
		float savedMotorSpeed = 0.f;
		float savedMaxMotorTorque = 0.f;

		double endTime = 0.0;

		bool wasAwake = true;
		float savedSleepThreshold = 0.f;
	};

	struct ExplosionAnim
	{
		b2Vec2 pos;
		double startTime;
		float radius;
		int damage;
		uint64_t ownerCharacterId;
	};

	// ---- VFX Leech Ray ----------------------------------------------------------
	struct LeechRay
	{
		b2Vec2 fromWorld;
		b2BodyId toBody;
		float t = 0.f;
		float T = 0.20f; // ← 0.1 s au lieu de 1.0f
		float w0 = 0.35f;
		float w1 = 0.05f;
		uint32_t rgba = 0xFFFFFFFF;
	};

	std::vector<LeechRay> m_leechRays;	// à mettre dans ta classe Game/World
	double m_lastLeechUpdateTime = 0.0; // dt pour les LeechRays

	struct HUDLines
	{
		std::string h1;										 // Weapon name
		std::string h2;										 // "Damage: X"
		std::string h3;										 // metric text
		std::string h4;										 // tagline
		ImU32 h1Color = IM_COL32( 255, 255, 255, 255 );		 // couleur perso (pour chiffres par défaut)
		ImU32 h1TitleColor = IM_COL32( 255, 255, 255, 255 ); // couleur du TITRE (grisée si mort)
		ImU32 h2DigitCol = IM_COL32( 255, 255, 255, 230 );	 // défaut: blanc
		ImU32 h3DigitCol = IM_COL32( 255, 255, 255, 230 );	 // défaut: blanc
		ImU32 h3TextCol = IM_COL32( 255, 255, 255, 230 );	 // par défaut: blanc
	};

	struct Seg
	{
		std::string t;
		bool num;
	};

	static constexpr uint16_t CATEGORY_WALL = 0x0001;
	static constexpr uint16_t CATEGORY_PROJECTILE = 0x0002;
	static constexpr uint16_t CATEGORY_CHARACTER = 0x0004;
	static constexpr uint16_t CATEGORY_WEAPON = 0x0008;
	static constexpr uint16_t CATEGORY_SKIN = 0x0010;
	static constexpr uint16_t CATEGORY_HITBOX = 0x0020;
	static constexpr uint16_t CATEGORY_TURRET = 0x0040;
	static constexpr uint16_t CATEGORY_KILLZONE = 0x0080;

	enum class ProjectileKind
	{
		Arrow,
		Firework,
		VampireKnife,
		Shuriken,
		Frost,
		Explosion,
		Electric,
		PoisonDart,
		Turret,
		// NEW
		Flask
	};

	// ───────────────────────────────────────────────────────────────────────────
	// Flask: couleurs nommées (plus de "LVL")
	// ───────────────────────────────────────────────────────────────────────────
	enum class FlaskColor : int
	{
		Red = 0,
		Orange,
		Brown,
		Yellow,
		Green,
		Cyan,
		Blue,
		Purple,
		Magenta,
		Grey,
		COUNT
	};

	// (Membre de classe) : presets & état de l'instance courante
	FlaskColor m_flaskColorPreset = FlaskColor::Red; // choisi dans l'UI
	FlaskColor m_flaskColor = FlaskColor::Red;		 // figé à la création

	// Noms lisibles pour l'UI (ImGui::Combo)
	static inline const char* kFlaskColorNames[(int)FlaskColor::COUNT] = { "Red",  "Orange", "Brown",  "Yellow",  "Green",
																		   "Cyan", "Blue",	 "Purple", "Magenta", "Grey" };

	// Helpers pour récupérer les noms d’assets exacts
	static inline const char* FlaskIconName( FlaskColor c )
	{
		switch ( c )
		{
			case FlaskColor::Red:
				return "FLASK_RED";
			case FlaskColor::Orange:
				return "FLASK_ORANGE";
			case FlaskColor::Brown:
				return "FLASK_BROWN";
			case FlaskColor::Yellow:
				return "FLASK_YELLOW";
			case FlaskColor::Green:
				return "FLASK_GREEN";
			case FlaskColor::Cyan:
				return "FLASK_CYAN";
			case FlaskColor::Blue:
				return "FLASK_BLUE";
			case FlaskColor::Purple:
				return "FLASK_PURPLE";
			case FlaskColor::Magenta:
				return "FLASK_MAGENTA";
			case FlaskColor::Grey:
				return "FLASK_GREY";
			default:
				return "FLASK_RED";
		}
	}

	static inline const char* FlaskProjectileName( FlaskColor c )
	{
		// Choisis un format et utilise-le partout à l’enregistrement des sprites.
		// Ici: "FLASK_PROJECTILE_<COLOR>"
		switch ( c )
		{
			case FlaskColor::Red:
				return "FLASK_PROJECTILE_RED";
			case FlaskColor::Orange:
				return "FLASK_PROJECTILE_ORANGE";
			case FlaskColor::Brown:
				return "FLASK_PROJECTILE_BROWN";
			case FlaskColor::Yellow:
				return "FLASK_PROJECTILE_YELLOW";
			case FlaskColor::Green:
				return "FLASK_PROJECTILE_GREEN";
			case FlaskColor::Cyan:
				return "FLASK_PROJECTILE_CYAN";
			case FlaskColor::Blue:
				return "FLASK_PROJECTILE_BLUE";
			case FlaskColor::Purple:
				return "FLASK_PROJECTILE_PURPLE";
			case FlaskColor::Magenta:
				return "FLASK_PROJECTILE_MAGENTA";
			case FlaskColor::Grey:
				return "FLASK_PROJECTILE_GREY";
			default:
				return "FLASK_PROJECTILE_RED";
		}
	}

	static constexpr float kCharacterRadius = 1.5f, kUnarmedRingPad = 0.1f, kPixelSize = 0.22f;
	static constexpr double kHitBlinkDuration = 0.25, kHitCooldown = 1.0, kHitCooldownDagger = 0.01;
	static constexpr int kMaxSpearLevel = 10;
	static constexpr double kHitFreezeDuration = 0.25, kMotorFlipCooldown = 1.0;
	static constexpr float kInstantFlipTargetSpeed = 10.0f;

	static constexpr uint32_t RGB( uint8_t r, uint8_t g, uint8_t b )
	{
		return ( uint32_t( r ) << 16 ) | ( uint32_t( g ) << 8 ) | uint32_t( b );
	}

	static constexpr int kPivotAnchorLevel = 10;
	static constexpr float kPivotL1 = 2.30f, kPivotL10 = 3.90f, kPivotSnapStep = 0.05f;

	using BodySet = std::unordered_set<b2BodyId, BodyIdHash>;

	// --- Clé orientée (victime, attaquant) ---
	struct PairKey
	{
		uint64_t victim;   // ← doit être le characterId (pas la skin/body brute)
		uint64_t attacker; // ← id du body attaquant (arme/projo/…)
	};
	struct PairKeyHash
	{
		size_t operator()( const PairKey& k ) const
		{
			size_t h1 = std::hash<uint64_t>{}( k.victim );
			size_t h2 = std::hash<uint64_t>{}( k.attacker );
			return h1 ^ ( h2 + 0x9e3779b97f4a7c15ULL + ( h1 << 6 ) + ( h1 >> 2 ) );
		}
	};
	struct PairKeyEq
	{
		bool operator()( const PairKey& a, const PairKey& b ) const
		{
			return a.victim == b.victim && a.attacker == b.attacker;
		}
	};

	// --- Clé symétrique (min, max) pour couples d'armes ---
	struct PairKeySym
	{
		uint64_t lo, hi; // lo <= hi
	};
	struct PairKeySymHash
	{
		size_t operator()( const PairKeySym& k ) const
		{
			size_t h1 = std::hash<uint64_t>{}( k.lo );
			size_t h2 = std::hash<uint64_t>{}( k.hi );
			return h1 ^ ( h2 + 0x9e3779b97f4a7c15ULL + ( h1 << 6 ) + ( h1 >> 2 ) );
		}
	};
	struct PairKeySymEq
	{
		bool operator()( const PairKeySym& a, const PairKeySym& b ) const
		{
			return a.lo == b.lo && a.hi == b.hi;
		}
	};

	// Helpers
	inline PairKey MakePairKey( uint64_t victimCharId, uint64_t attacker ) const
	{
		return PairKey{ victimCharId, attacker };
	}
	inline PairKeySym MakePairKeySym( uint64_t a, uint64_t b ) const
	{
		return ( a <= b ) ? PairKeySym{ a, b } : PairKeySym{ b, a };
	}

	// ========= API Sample =========

	static Sample* Create( SampleContext* context )
	{
		return new WeaponsBallsVS( context );
	}
	explicit WeaponsBallsVS( SampleContext* c )
		: Sample( c )
	{
		ApplyGravitySetting();

		if ( !m_context->restart )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 20.0f;
		}

		InitAudioBanks();

		m_currentSeed1v1 = std::random_device{}();
		m_rng1v1.seed( m_currentSeed1v1 );

		ApplyArenaPreset( m_arenaPreset );

		ClearAllCharacters();

		m_unarmedCfg.ghostMinDist = 0.01f;
		m_unarmedCfg.ghostTTL = 1.0; // plus long
		m_unarmedCfg.ghostMax = 10;
		m_unarmedCfg.ghostRadius = kCharacterRadius + kUnarmedRingPad;

		ApplyUnarmedTuning();
		CreateGround();

		if ( m_selectedCharIdx1 < 0 || m_selectedCharIdx1 >= (int)kAllCharacters.size() )
			m_selectedCharIdx1 = 0;
		if ( m_selectedCharIdx2 < 0 || m_selectedCharIdx2 >= (int)kAllCharacters.size() )
			m_selectedCharIdx2 = 1;

		const std::string& char1 = kAllCharacters[m_selectedCharIdx1];
		const std::string& char2 = kAllCharacters[m_selectedCharIdx2];
		SpawnSelectedCharacters( char1, char2, m_rng1v1 );
	}

	// ========= API publique (extrait) =========

	void ApplyGravitySetting()
	{
		if ( !B2_IS_NON_NULL( m_worldId ) )
			return;
		const b2Vec2 g = m_gravityEnabled ? b2Vec2{ 0.0f, m_gravityYParam } : b2Vec2{ 0.0f, 0.0f };
		b2World_SetGravity( m_worldId, g );
	}

	void ApplyUnarmedTuning()
	{
		m_unarmedMinVel = m_unarmedCfg.minVel;
		m_unarmedMinVelCap = m_unarmedCfg.minVelCap;
		m_unarmedMinVelGrow = m_unarmedCfg.minVelGrow;

		m_unarmedGhostRadius = m_unarmedCfg.ghostRadius;
		m_unarmedGhostMinDist = m_unarmedCfg.ghostMinDist;
		m_unarmedGhostTTL = m_unarmedCfg.ghostTTL;
		m_unarmedGhostMax = m_unarmedCfg.ghostMax;
	}

	void ApplyArenaPreset( int idx )
	{
		m_arenaPreset = std::clamp( idx, 0, 2 );
		switch ( m_arenaPreset )
		{
			default:
			case 0:
				m_arenaHalfWidthParam = 10.0f;
				m_arenaHalfHeightParam = 10.0f;
				m_wallHalfThicknessParam = 0.10f;
				m_killOuterPadX = 10.0f;
				m_killOuterPadY = 20.0f;
				break;
			case 1:
				m_arenaHalfWidthParam = 20.0f;
				m_arenaHalfHeightParam = 20.0f;
				m_wallHalfThicknessParam = 0.10f;
				m_killOuterPadX = 10.0f;
				m_killOuterPadY = 35.0f;
				break;
			case 2:
				m_arenaHalfWidthParam = 40.0f;
				m_arenaHalfHeightParam = 40.0f;
				m_wallHalfThicknessParam = 0.10f;
				m_killOuterPadX = 20.0f;
				m_killOuterPadY = 60.0f;
				break;
		}

		if ( m_context )
			m_context->camera.zoom = ZoomForPreset( m_arenaPreset );
	}

	void SetWallThickness( float totalThickness )
	{
		m_wallHalfThicknessParam = 0.5f * totalThickness; // total = 2*t

		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
			m_groundId = b2_nullBodyId;
		}
		CreateGround(); // Recrée les 4 murs + kill sensor
	}

	void CreateGround()
	{
		const float arenaHalfWidth = m_arenaHalfWidthParam;
		const float arenaHalfHeight = m_arenaHalfHeightParam;
		const float t = m_wallHalfThicknessParam;
		const float chamfer = std::min( 0.02f, 0.45f * t ); // au lieu du 0.02f fixe
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_staticBody;
		bd.position = { 0.0f, 0.0f };
		m_groundId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.friction = 0.0f;
		sd.material.restitution = 0.0f;
		sd.material.customColor = 0x1A1A1A;

		sd.filter = b2DefaultFilter();
		sd.filter.categoryBits = CATEGORY_WALL;
		sd.filter.maskBits = B2_DEFAULT_MASK_BITS;

		sd.enableContactEvents = true;

		sd.invokeContactCreation = false;
		sd.updateBodyMass = false;

		{
			const b2Vec2 center = { 0.0f, -arenaHalfHeight + t };
			b2Polygon box = b2MakeOffsetRoundedBox( arenaHalfWidth, t, center, b2Rot_identity, chamfer );
			(void)b2CreatePolygonShape( m_groundId, &sd, &box );
		}
		{
			const b2Vec2 center = { -arenaHalfWidth + t, 0.0f };
			b2Polygon box = b2MakeOffsetRoundedBox( t, arenaHalfHeight, center, b2Rot_identity, chamfer );
			(void)b2CreatePolygonShape( m_groundId, &sd, &box );
		}
		{
			const b2Vec2 center = { +arenaHalfWidth - t, 0.0f };
			b2Polygon box = b2MakeOffsetRoundedBox( t, arenaHalfHeight, center, b2Rot_identity, chamfer );
			(void)b2CreatePolygonShape( m_groundId, &sd, &box );
		}
		{
			const b2Vec2 center = { 0.0f, +arenaHalfHeight - t };
			b2Polygon box = b2MakeOffsetRoundedBox( arenaHalfWidth, t, center, b2Rot_identity, chamfer );
			(void)b2CreatePolygonShape( m_groundId, &sd, &box );
		}

		ComputeKillAABB();
		CreateKillSensorLoop();
	}

	void CreateKillSensorLoop()
	{
		if ( B2_IS_NON_NULL( m_killBodyId ) )
		{
			b2DestroyBody( m_killBodyId );
			m_killBodyId = b2_nullBodyId;
		}

		const float W = m_arenaHalfWidthParam;
		const float H = m_arenaHalfHeightParam;
		const float t = m_wallHalfThicknessParam;

		const float xOut = W + t + m_killOuterPadX;
		const float yOut = H + t + m_killOuterPadY;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_staticBody;
		bd.position = { 0.0f, 0.0f };
		m_killBodyId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.isSensor = true;
		sd.enableSensorEvents = true;
		sd.filter.categoryBits = CATEGORY_KILLZONE;
		sd.filter.maskBits = CATEGORY_PROJECTILE; // on ne tue que les projectiles
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.customColor = 0xFF0000; // rouge fin overlay
		const float fence = std::max( 0.05f, 0.5f * m_wallHalfThicknessParam );
		{
			b2Polygon box = b2MakeOffsetBox( xOut, fence, { 0.0f, -yOut }, b2Rot_identity );
			(void)b2CreatePolygonShape( m_killBodyId, &sd, &box );
		}
		{
			b2Polygon box = b2MakeOffsetBox( xOut, fence, { 0.0f, +yOut }, b2Rot_identity );
			(void)b2CreatePolygonShape( m_killBodyId, &sd, &box );
		}
		{
			b2Polygon box = b2MakeOffsetBox( fence, yOut, { -xOut, 0.0f }, b2Rot_identity );
			(void)b2CreatePolygonShape( m_killBodyId, &sd, &box );
		}
		{
			b2Polygon box = b2MakeOffsetBox( fence, yOut, { +xOut, 0.0f }, b2Rot_identity );
			(void)b2CreatePolygonShape( m_killBodyId, &sd, &box );
		}
	}

	void ComputeKillAABB()
	{
		const float W = m_arenaHalfWidthParam;
		const float H = m_arenaHalfHeightParam;
		const float t = m_wallHalfThicknessParam;

		m_killXmin = -( W + t + m_killOuterPadX );
		m_killXmax = +( W + t + m_killOuterPadX );
		m_killYmin = -( H + t + m_killOuterPadY );
		m_killYmax = +( H + t + m_killOuterPadY );
	}

	void ClearAllCharacters()
	{
		// Détruire/recréer le monde
		if ( B2_IS_NON_NULL( m_worldId ) )
			b2DestroyWorld( m_worldId );
		m_worldId = b2_nullWorldId;

		b2WorldDef worldDef = b2DefaultWorldDef();
		m_worldId = b2CreateWorld( &worldDef );
		b2World_SetContactTuning( m_worldId, 240.0f, 0.01f, 1.0f );
		ApplyGravitySetting();

		// Tables gameplay / mappings
		m_characterHP.clear();
		m_characterWeapon.clear();
		m_weaponDamage.clear();
		m_weaponOwner.clear();
		m_shapeToCharacter.clear();
		m_projectilesToDestroyMap.clear();
		m_lastPairSwitchTime.clear();
		m_poisonBuildUp.clear();
		m_shapeBaseColor.clear();
		m_weaponToJoint.clear();
		m_characterSkinShape.clear();
		m_activeExplosions.clear();

		// Projectiles sets
		m_projectileArrows.clear();
		m_projectileFireworks.clear();
		m_projectileVampireKnives.clear();
		m_projectileShuriken.clear();
		m_projectileFrost.clear();
		m_projectileExplosion.clear();
		m_projectileElectricStaff.clear();
		m_projectilePoisonDarts.clear();
		m_projectileTurrets.clear();

		// Contacts / états transitoires
		m_weaponPairContactTime.clear();
		m_electricStaffFreezeDuration.clear();
		m_damageCooldown.clear();
		m_shurikenReboundsLeft.clear();
		m_lastHitBlinkTime.clear();
		m_vampireKnifeHealCount.clear();
		m_vampireKnifeTotalHealed.clear();

		// Tourelles
		m_turretIds.clear();
		m_turretOwner.clear();
		m_turretLastShot.clear();

		// Spawns / destructions / SFX
		m_projectileSpawnPos.clear();
		m_charactersToKill.clear();
		m_nextReboundSoundAllowed.clear();
		m_sfxSeenProjectiles.clear();

		// Ghost trail Unarmed
		m_unarmedGhosts.clear();
		m_unarmedGhostInit = false;
		m_unarmedLastGhostPos = { 0.f, 0.f }; // NEW

		// Sol / arène
		m_groundId = b2_nullBodyId;
		m_arenaWalls.clear(); // NEW

		// Armes (bodies)
		m_bowId = m_crossbowId = m_swordId = m_axeId = m_daggerId = m_katanaId = m_tridentId = m_hammerId = m_poisonBlowgunId =
			m_clubId = m_scytheId = m_spearId = m_kunaiId = m_boomerangId = m_shurikenId = m_bigSwordId = m_electricStaffId =
				m_explosionStaffId = m_frostStaffId = m_mahoragaId = m_vampireKnifeId = m_shieldId = m_wrenchId =
					/* AJOUT */ m_glaiveId = m_sickleRId = m_flaskId = b2_nullBodyId,
		m_lanceId = b2_nullBodyId;

		// Burst arc
		m_arrowBurst = ArrowBurst{};

		// Personnages (bodies)
		m_characterIdBow = m_characterIdCrossbowBow = m_characterIdSword = m_characterIdAxe = m_characterIdDagger =
			m_characterIdKatana = m_characterIdTrident = m_characterIdhammer = m_characterIdPoisonBlowgun = m_characterIdClub =
				m_characterIdScythe = m_characterIdSpear = m_characterIdKunai = m_characterIdBoomerang = m_characterIdShuriken =
					m_characterIdBigSword = m_characterIdElectricStaff = m_characterIdExplosionStaff = m_characterIdFrostStaff =
						m_characterIdMahoraga = m_characterIdShield = m_characterIdVampireKnife = m_characterIdWrench =
							/* AJOUT */ m_characterIdGlaive = m_characterIdSickleR = m_characterIdFlask = b2_nullBodyId,
		m_characterIdUnarmed = b2_nullBodyId, m_characterIdLance = b2_nullBodyId;

		// Joints
		m_bowJointId = m_crossbowJointId = m_swordJointId = m_axeJointId = m_daggerJointId = m_katanaJointId = m_tridentJointId =
			m_hammerJointId = m_poisonBlowgunJointId = m_clubJointId = m_scytheJointId = m_spearJointId = m_kunaiJointId =
				m_boomerangJointId = m_shurikenJointId = m_bigSwordJointId = m_electricStaffJointId = m_explosionStaffJointId =
					m_frostStaffJointId = m_mahoragaJointId = m_vampireKnifeJointId = m_shieldJointId = m_wrenchJointId =
						/* AJOUT */ m_glaiveJointId = m_sickleRJointId = m_flaskJointId = b2_nullJointId,
		m_lanceJointId = b2_nullJointId;

		// Kill zone
		m_killBodyId = b2_nullBodyId;
		m_killXmin = m_killXmax = m_killYmin = m_killYmax = 0;

		// Timers projectiles
		m_lastArrowTime = 0.;
		m_lastFireworkTime = 0.;
		m_lastVampireKnifeTime = 0.;
		m_lastShurikenTime = 0.;
		m_lastElectricStaffTime = 0.;
		m_lastExplosionTime = -10.0;
		m_lastFrostTime = 0.;
		m_lastPoisonDartTime = 0.;
		m_crossbowVolleyCount = 10;

		// Freezes & cooldowns
		m_activeFreezes.clear();
		m_damageCooldown.clear(); // (ok si doublon)

		// Ramps / stacks / progressions
		m_scythePoisonStacks = 1;
		m_katanaSlashStacks = 1;
		m_hammerIncInterval = 1.0;
		m_hammerJustInverted = false;
		m_hammerLastIncTime = 0.0;

		m_boomerangReachA = 1.0f;
		m_boomerangReachB = 1.0f;
		m_boomerangAnimSpeed = 1.0f;
		m_boomerangHitCount = 0;

		m_explosionRadius = 2.f;
		m_explosionMagnitude = 1.f;
		m_explosionDamage = 1;

		m_shurikenBonusRebounds = 0;
		m_bowAutoInterval = 0.01;
		m_daggerHitCooldown = kHitCooldownDagger;

		m_spearLevel = 1;
		m_spearPixelSize = 0.20f;

		m_unarmedLastT = 0.0;

		// DoT / FX slash
		m_slashBuildUp.clear();
		m_slashLastTick.clear();
		m_activeSlashLines.clear();
		m_slashTickInterval = 0.1;
		m_slashFxDuration = 0.10;

		m_leechRays.clear();
		m_lastLeechUpdateTime = 0.0;

		// Paires / collisions
		m_pairOverlap.clear();
		m_flipLatched.clear(); // NEW

		// Boomerangs anim par-weapon
		m_boomerangs.clear();

		// ⬇️ (nouveau) Réinitialisation **runtime** d’Unarmed d’après la config
		ApplyUnarmedTuning();	   // NEW : remet m_unarmedMinVel/cap/grow + ghost params
		m_lastUnarmedIncStep = -1; // NEW : repart propre pour l’auto-incrément sur hit

		m_projectileFlask.clear(); // <- en plus des autres

		// Timers projectiles
		m_lastFlaskTime = 0.0; // <- en plus des autres
	}

	void SpawnAllCharacters( std::mt19937& rng )
	{
		auto positions = GenerateSpawnPositionsNoJitter( (int)kAllCharacters.size(), 15.f, { 0, 0 } );
		for ( size_t i = 0; i < kAllCharacters.size(); ++i )
		{
			const b2Vec2& pos = positions[(int)i];
			const std::string& name = kAllCharacters[i];
			if ( auto it = spawnFuncs.find( name ); it != spawnFuncs.end() )
				( this->*( it->second ) )( pos );
		}
		RandomLaunchAllCharacters( rng );
	}

	void SpawnSelectedCharacters( const std::string& char1, const std::string& char2, std::mt19937& rng )
	{
		std::vector<std::string> selected{ char1, char2 };
		auto positions = GenerateSpawnPositionsNoJitter( (int)selected.size(), 7.f, { 0, 0 } );
		for ( size_t i = 0; i < selected.size(); ++i )
		{
			const b2Vec2& pos = positions[(int)i];
			const std::string& name = selected[i];
			if ( auto it = spawnFuncs.find( name ); it != spawnFuncs.end() )
				( this->*( it->second ) )( pos );
		}
		RandomLaunchAllCharacters( rng );
	}

	void AutoFireAll( double now )
	{
		if ( B2_IS_NON_NULL( m_bowId ) )
		{
			if ( m_arrowBurst.active )
			{
				if ( now >= m_arrowBurst.nextTime && m_arrowBurst.shot < m_arrowBurst.total )
				{
					FireBowProjectileFromCurrent();
					m_arrowBurst.shot++;
					m_arrowBurst.nextTime = now + 0.015;
				}
				if ( m_arrowBurst.shot >= m_arrowBurst.total )
				{
					m_arrowBurst.active = false;
					m_lastArrowTime = now;
				}
			}
			else if ( now - m_lastArrowTime > 1.0 )
			{
				m_arrowBurst.active = true;
				m_arrowBurst.shot = 0;
				m_arrowBurst.nextTime = now;
			}
		}

		if ( now - m_lastFireworkTime > 1.0 && B2_IS_NON_NULL( m_crossbowId ) )
		{
			FireFireworkFromCrossbow();
			m_lastFireworkTime = now;
		}
		if ( now - m_lastVampireKnifeTime > 1.0 && B2_IS_NON_NULL( m_vampireKnifeId ) )
		{
			FireVampireKnifeProjectile();
			m_lastVampireKnifeTime = now;
		}
		if ( now - m_lastShurikenTime > 0.7 && B2_IS_NON_NULL( m_shurikenId ) )
		{
			FireShurikenFromCurrent();
			m_lastShurikenTime = now;
		}
		if ( now - m_lastFrostTime > 1.0 && B2_IS_NON_NULL( m_frostStaffId ) )
		{
			FireFrostProjectileFromCurrent();
			m_lastFrostTime = now;
		}
		if ( now - m_lastExplosionTime > 1.0 && B2_IS_NON_NULL( m_explosionStaffId ) )
		{
			FireExplosionProjectileFromCurrent();
			m_lastExplosionTime = now;
		}
		if ( now - m_lastElectricStaffTime > 0.5 && B2_IS_NON_NULL( m_electricStaffId ) )
		{
			FireElectricStaffProjectileFromCurrent();
			m_lastElectricStaffTime = now;
		}
		if ( now - m_lastPoisonDartTime > 0.5 && B2_IS_NON_NULL( m_poisonBlowgunId ) )
		{
			FirePoisonDartFromCurrent();
			m_lastPoisonDartTime = now;
		}

		if ( now - m_lastFlaskTime > 1.0 && B2_IS_NON_NULL( m_flaskId ) )
		{
			FireFlaskProjectileFromCurrent();
			m_lastFlaskTime = now;
		}
	}

	void MaintainMinVelocityForAll()
	{
		for ( const std::string& name : kAllCharacters )
		{
			b2BodyId id = GetCharacterIdByName( name );
			if ( !BodyValid( id ) )
				continue;
			if ( IsBodyCurrentlyFrozen( id ) )
				continue; // ← ton skip : OK
			if ( b2Body_GetType( id ) != b2_dynamicBody )
				continue; // ← dyn only

			float minV = ( id == m_characterIdUnarmed ) ? m_unarmedMinVel : m_minSpeedAll;

			b2Vec2 v = b2Body_GetLinearVelocity( id );
			float v2 = v.x * v.x + v.y * v.y;
			if ( v2 < minV * minV )
				EnsureMinVelocity( id, minV ); // garde la direction; si |v|≈0, choisis une direction par défaut
		}
	}

	void ProcessHitSensors()
	{
		b2SensorEvents sensorEvents = b2World_GetSensorEvents( m_worldId );

		// ─────────────────────────────────────────────────────────────────────────────
		// BEGIN TOUCH (entrées dans sensors / shapes avec enableSensorEvents)
		// ─────────────────────────────────────────────────────────────────────────────
		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			const b2SensorBeginTouchEvent& evt = sensorEvents.beginEvents[i];
			if ( !b2Shape_IsValid( evt.sensorShapeId ) || !b2Shape_IsValid( evt.visitorShapeId ) )
				continue;

			// Killzone -> proj only
			{
				b2ShapeId sA = evt.sensorShapeId;
				b2ShapeId sB = evt.visitorShapeId;
				b2Filter fA = b2Shape_GetFilter( sA );
				b2Filter fB = b2Shape_GetFilter( sB );

				const bool AisKill = ( fA.categoryBits & CATEGORY_KILLZONE ) != 0;
				const bool BisKill = ( fB.categoryBits & CATEGORY_KILLZONE ) != 0;
				const bool AisProj = ( fA.categoryBits & CATEGORY_PROJECTILE ) != 0;
				const bool BisProj = ( fB.categoryBits & CATEGORY_PROJECTILE ) != 0;

				if ( ( AisKill && BisProj ) || ( BisKill && AisProj ) )
				{
					b2ShapeId projShape = AisProj ? sA : sB;
					b2BodyId projBody = b2Shape_GetBody( projShape );
					if ( BodyValid( projBody ) )
						ScheduleProjectileDestroy( projBody, ImGui::GetTime() );
					continue;
				}
			}

			b2ShapeId shapeA = evt.sensorShapeId;
			b2ShapeId shapeB = evt.visitorShapeId;

			b2BodyId bodyA = b2Shape_GetBody( shapeA );
			b2BodyId bodyB = b2Shape_GetBody( shapeB );
			if ( !BodyValid( bodyA ) || !BodyValid( bodyB ) )
				continue;

			b2Filter filterA = b2Shape_GetFilter( shapeA );
			b2Filter filterB = b2Shape_GetFilter( shapeB );

			// Petit filtre anti "armes collées" (armes physiques non projectiles)
			bool aIsPhysWeapon = ( filterA.categoryBits & CATEGORY_WEAPON ) && !( filterA.categoryBits & CATEGORY_PROJECTILE );
			bool bIsPhysWeapon = ( filterB.categoryBits & CATEGORY_WEAPON ) && !( filterB.categoryBits & CATEGORY_PROJECTILE );
			if ( aIsPhysWeapon && bIsPhysWeapon )
			{
				uint64_t idA = b2StoreBodyId( bodyA ), idB = b2StoreBodyId( bodyB );
				PairKeySym pairKeyStick = MakePairKeySym( idA, idB );
				auto it = m_weaponPairContactTime.find( pairKeyStick );
				if ( it != m_weaponPairContactTime.end() )
				{
					double stuckDuration = ImGui::GetTime() - it->second;
					if ( stuckDuration > 0.1 )
						continue;
				}
			}

			const bool aIsSkin = ( filterA.categoryBits & CATEGORY_SKIN ) != 0;
			const bool bIsSkin = ( filterB.categoryBits & CATEGORY_SKIN ) != 0;
			const bool aIsWeaponOrProj = ( filterA.categoryBits & ( CATEGORY_WEAPON | CATEGORY_PROJECTILE ) ) != 0;
			const bool bIsWeaponOrProj = ( filterB.categoryBits & ( CATEGORY_WEAPON | CATEGORY_PROJECTILE ) ) != 0;
			if ( !( ( aIsSkin && bIsWeaponOrProj ) || ( bIsSkin && aIsWeaponOrProj ) ) )
				continue;

			b2BodyId skinBody = aIsSkin ? bodyA : bodyB;
			b2BodyId attackerBody = aIsSkin ? bodyB : bodyA;

			// ── Résoudre la victime au niveau "character" (via la shape SKIN) ──
			const uint64_t victimSkinShapeId = b2StoreShapeId( aIsSkin ? shapeA : shapeB );
			auto itVictimMap = m_shapeToCharacter.find( victimSkinShapeId );
			if ( itVictimMap == m_shapeToCharacter.end() )
				continue; // pas de mapping => on ignore
			const uint64_t victimCharId = itVictimMap->second;

			const uint64_t attackerId = b2StoreBodyId( attackerBody );
			PairKey pk = MakePairKey( victimCharId, attackerId );
			uint64_t weaponId = attackerId;

			// Pas d'auto-hit (owner == victim) ni de tourelle qui tape son owner
			if ( auto itOwner = m_weaponOwner.find( weaponId ); itOwner != m_weaponOwner.end() )
			{
				b2BodyId owner = itOwner->second;
				if ( auto itTurret = m_turretOwner.find( owner ); itTurret != m_turretOwner.end() )
				{
					if ( b2StoreBodyId( itTurret->second ) == victimCharId )
						continue;
				}
				else if ( b2StoreBodyId( owner ) == victimCharId )
				{
					continue;
				}
			}

			// ─────────────────────────────────────────────────────────────
			// Gating "body complet": compteur d'overlaps pour la paire
			// ─────────────────────────────────────────────────────────────
			int& overlap = m_pairOverlap[pk];
			const bool firstEnter = ( overlap == 0 );
			++overlap; // on incrémente à chaque Begin de fixture; décrémenté au EndTouch

			// ─────────────────────────────────────────────────────────────
			// Cooldown & autorisation de dégâts
			// ─────────────────────────────────────────────────────────────
			const double now = ImGui::GetTime();
			double cooldown = kHitCooldown;
			if ( weaponId == b2StoreBodyId( m_daggerId ) )
				cooldown = m_daggerHitCooldown;

			double& nextAllowedTime = m_damageCooldown[pk];
			const bool allowDamage = firstEnter && ( now >= nextAllowedTime );
			if ( allowDamage )
				nextAllowedTime = now + cooldown;

			// ─────────────────────────────────────────────────────────────
			// Application des dégâts/effets (UNIQUEMENT si allowDamage)
			// ─────────────────────────────────────────────────────────────
			if ( allowDamage )
			{
				auto itHp = m_characterHP.find( victimCharId );
				if ( itHp != m_characterHP.end() )
				{
					int dmg = 0;
					if ( auto itDmg = m_weaponDamage.find( weaponId ); itDmg != m_weaponDamage.end() )
						dmg = itDmg->second;

					// PATCH : pas de dégâts au contact pour les projectiles d'explosion (gérés par TriggerExplosion)
					if ( m_projectileExplosion.count( attackerBody ) )
						dmg = 0;

					int& hp = itHp->second;

					if ( hp > 0 && dmg > 0 )
					{
						const int hpBefore = hp;
						hp = std::max( 0, hp - dmg );

						if ( hp < hpBefore )
						{
							m_lastHitBlinkTime[victimCharId] = ImGui::GetTime();

							const bool isProjectileBody =
								m_projectileArrows.count( attackerBody ) || m_projectileFireworks.count( attackerBody ) ||
								m_projectileVampireKnives.count( attackerBody ) || m_projectileShuriken.count( attackerBody ) ||
								m_projectileFrost.count( attackerBody ) || m_projectileExplosion.count( attackerBody ) ||
								m_projectileElectricStaff.count( attackerBody ) ||
								m_projectilePoisonDarts.count( attackerBody ) || m_projectileTurrets.count( attackerBody ) ||
								m_projectileFlask.count( attackerBody ); // <- AJOUT

							b2Vec2 impactPos = b2Body_GetPosition( skinBody );
							if ( !isProjectileBody )
							{
								PlayMeleeDamageSound( attackerBody, impactPos );
							}
							else
							{
								float spd = 0.f;
								if ( BodyValid( attackerBody ) )
								{
									b2Vec2 v = b2Body_GetLinearVelocity( attackerBody );
									spd = b2Length( v );
								}
								PlayProjectileImpactSound( impactPos, spd ); // son unique pour tous les projectiles
							}

							// Shuriken : bonus de rebonds selon dégâts infligés
							if ( m_projectileShuriken.count( attackerBody ) )
							{
								int hpLost = hpBefore - hp;
								if ( hpLost > 0 )
									m_shurikenBonusRebounds += hpLost;
								m_projectilesToDestroyMap[attackerBody] = now;
							}

							// hammer (attaquant)
							if ( weaponId == b2StoreBodyId( m_hammerId ) )
							{
								m_hammerIncInterval = std::max( 0.1, m_hammerIncInterval - 0.1 );
								if ( auto it = m_weaponDamage.find( weaponId ); it != m_weaponDamage.end() )
									it->second = 1;
								m_hammerLastIncTime = now;
							}
							// hammer (victime)
							if ( auto ithammer = m_characterWeapon.find( victimCharId );
								 ithammer != m_characterWeapon.end() && ithammer->second == m_hammerId )
							{
								if ( auto ithammerDmg = m_weaponDamage.find( b2StoreBodyId( m_hammerId ) );
									 ithammerDmg != m_weaponDamage.end() )
									ithammerDmg->second = 1;
								m_hammerLastIncTime = now;
							}

							// Katana → Slash DoT
							if ( weaponId == b2StoreBodyId( m_katanaId ) )
								AddSlash( victimCharId, std::max( 1, m_katanaSlashStacks ), now );

							// Scythe → Poison DoT
							if ( weaponId == b2StoreBodyId( m_scytheId ) )
								AddPoison( victimCharId, std::max( 1, m_scythePoisonStacks ), now );

							// Poison Darts → build-up + destruction projectile
							if ( m_projectilePoisonDarts.count( attackerBody ) )
							{
								auto& p = m_poisonBuildUp[victimCharId];
								if ( p.second <= 0 )
									p.first = now;
								p.second += m_poisonDartStacks;
								m_projectilesToDestroyMap[attackerBody] = ImGui::GetTime();
							}

							// Axe → scaling
							if ( weaponId == b2StoreBodyId( m_axeId ) )
								if ( auto itAxe = m_weaponDamage.find( weaponId );
									 itAxe != m_weaponDamage.end() && itAxe->second > 0 )
									itAxe->second = std::max( 1, (int)std::ceil( itAxe->second * 1.5 ) );

							// Dagger → accélère le moteur + réduit cooldown
							if ( weaponId == b2StoreBodyId( m_daggerId ) )
							{
								if ( B2_IS_NON_NULL( m_daggerJointId ) )
								{
									float speed = b2RevoluteJoint_GetMotorSpeed( m_daggerJointId );
									float sign = ( speed >= 0.f ) ? 1.f : -1.f;
									float newSpeed = std::clamp( speed + 2.0f * sign, -1000.0f, 1000.0f );
									b2RevoluteJoint_SetMotorSpeed( m_daggerJointId, newSpeed );
								}

								// ↓↓↓ NEW: réduire le cooldown après chaque hit qui fait perdre des HP
								constexpr double kDaggerCDMin = 0.002;	 // plancher (2 ms)
								constexpr double kDaggerCDFactor = 0.85; // -15% après chaque hit
								m_daggerHitCooldown = std::max( kDaggerCDMin, m_daggerHitCooldown * kDaggerCDFactor );
							}
							// Arc → chain burst
							if ( m_projectileArrows.count( attackerBody ) )
								m_arrowBurst.total = std::min( m_arrowBurst.total + 1, 100 );

							// Crossbow → +1 projectile simultané sur le prochain tir
							if ( m_projectileFireworks.count( attackerBody ) )
							{
								m_crossbowVolleyCount = std::min( m_crossbowVolleyCount + 1, 100 );
							}

							// Vampire Knife → vol de vie + scaling des dégâts
							if ( m_projectileVampireKnives.count( attackerBody ) )
							{
								if ( auto itOwner2 = m_weaponOwner.find( attackerId ); itOwner2 != m_weaponOwner.end() )
								{
									b2BodyId ownerId = itOwner2->second;
									if ( auto itOwnerHp = m_characterHP.find( b2StoreBodyId( ownerId ) );
										 itOwnerHp != m_characterHP.end() )
									{
										int& healCount = m_vampireKnifeHealCount[b2StoreBodyId( ownerId )];
										int& totalHealed = m_vampireKnifeTotalHealed[b2StoreBodyId( ownerId )];
										int healAmount = std::max( 1, dmg / ( 1 + healCount ) );
										int hpBeforeHeal = itOwnerHp->second;
										itOwnerHp->second = std::min( 100, itOwnerHp->second + healAmount );
										(void)hpBeforeHeal;
										healCount++;
										totalHealed += healAmount;
										int baseDmg = 1 + totalHealed / 10;
										m_weaponDamage[b2StoreBodyId( m_vampireKnifeId )] = std::max( baseDmg, 1 );
									}
								}
							}
							// --- VFX Leech Ray pour VampireKnives ---------------------------------
							if ( m_projectileVampireKnives.count( attackerBody ) )
							{
								// owner = perso qui a tiré le projectile (m_weaponOwner sur le body du projo)
								b2BodyId owner = b2_nullBodyId;
								auto itOwner = m_weaponOwner.find( attackerId );
								if ( itOwner != m_weaponOwner.end() )
									owner = itOwner->second;

								// point d’impact
								b2Vec2 hitP = impactPos;

								if ( B2_IS_NON_NULL( owner ) )
								{
									SpawnLeechRay( hitP, owner );
								}
							}
							// ----------------------------------------------------------------------

							// Turret projectile → se détruit à l'impact
							if ( m_projectileTurrets.count( attackerBody ) )
								m_projectilesToDestroyMap[attackerBody] = ImGui::GetTime();

							// Wrench → invoque une tourelle
							if ( weaponId == b2StoreBodyId( m_wrenchId ) )
							{
								b2Vec2 wrenchPos = b2Body_GetPosition( m_characterIdWrench );
								float offsetDist = 2.5f;
								static std::uniform_int_distribution<int> dirDist( 0, 3 );
								int direction = dirDist( m_rngAll );

								b2Vec2 offset;
								switch ( direction )
								{
									case 0:
										offset = { 0.0f, offsetDist };
										break;
									case 1:
										offset = { 0.0f, -offsetDist };
										break;
									case 2:
										offset = { -offsetDist, 0.0f };
										break;
									case 3:
										offset = { offsetDist, 0.0f };
										break;
									default:
										offset = { 0.0f, -offsetDist };
								}
								SummonTurretFromWrench( wrenchPos + offset );
							}

							double freezeDuration = isProjectileBody ? m_projectileHitFreeze : kHitFreezeDuration;

							// Unarmed → progression de vitesse min
							if ( attackerBody == m_characterIdUnarmed && hpBefore > hp )
							{
								if ( m_lastUnarmedIncStep != m_stepCounter )
								{
									m_unarmedMinVel = std::min( m_unarmedMinVel + m_unarmedMinVelGrow, m_unarmedMinVelCap );
									m_lastUnarmedIncStep = m_stepCounter;
								}
							}

							// Passifs de l'arme
							UpdateWeaponsPassives( weaponId );

							// ─────────────────────────────────────────────
							// Hit-freeze victime (PATCH Unarmed ici)
							// ─────────────────────────────────────────────
							if ( !IsBodyCurrentlyFrozen( skinBody ) )
							{
								const bool attackerIsUnarmed = ( attackerBody == m_characterIdUnarmed );

								b2JointId victimJoint = b2_nullJointId;
								if ( !attackerIsUnarmed )
								{
									if ( auto itVictimJoint = m_characterWeapon.find( victimCharId );
										 itVictimJoint != m_characterWeapon.end() )
									{
										b2BodyId victimWeapon = itVictimJoint->second;
										if ( auto itJoint = m_weaponToJoint.find( b2StoreBodyId( victimWeapon ) );
											 itJoint != m_weaponToJoint.end() )
											victimJoint = itJoint->second;
									}
								}
								// 👉 si Unarmed attaque, victimJoint reste null → moteur NON désactivé
								FreezeBodyAndJoint( skinBody, victimJoint, freezeDuration );
							}

							// Hit-freeze attaquant (si non projectile)
							b2Filter attackerFilter = {};
							b2ShapeId attackerShapes[8];
							int nA = b2Body_GetShapes( attackerBody, attackerShapes, 8 );
							if ( nA > 0 )
								attackerFilter = b2Shape_GetFilter( attackerShapes[0] );

							if ( ( attackerFilter.categoryBits & CATEGORY_PROJECTILE ) == 0 )
							{
								// Char owner
								b2BodyId attackerCharBody = b2_nullBodyId;
								if ( auto itOwner2 = m_weaponOwner.find( attackerId ); itOwner2 != m_weaponOwner.end() )
									attackerCharBody = itOwner2->second;

								if ( B2_IS_NON_NULL( attackerCharBody ) && !IsBodyCurrentlyFrozen( attackerCharBody ) )
									FreezeBodyAndJoint( attackerCharBody, b2_nullJointId, kHitFreezeDuration );

								// Arme (ou le body lui-même)
								b2BodyId attackerWeaponBody = b2_nullBodyId;
								if ( auto itWeaponBody = m_characterWeapon.find( attackerId );
									 itWeaponBody != m_characterWeapon.end() )
									attackerWeaponBody = itWeaponBody->second;
								if ( !B2_IS_NON_NULL( attackerWeaponBody ) )
									attackerWeaponBody = attackerBody;

								if ( !IsBodyCurrentlyFrozen( attackerWeaponBody ) )
								{
									b2JointId attackerJoint = b2_nullJointId;
									if ( auto itAttackerJoint = m_weaponToJoint.find( b2StoreBodyId( attackerWeaponBody ) );
										 itAttackerJoint != m_weaponToJoint.end() )
										attackerJoint = itAttackerJoint->second;

									FreezeBodyAndJoint( attackerWeaponBody, attackerJoint, kHitFreezeDuration );
								}
							}

							// Mort ?
							if ( hp == 0 )
								RequestKillCharacter( skinBody );
						}
					}
				}
			} // if (allowDamage)

			// ─────────────────────────────────────────────────────────────
			// Traitement des projectiles (destruction/rebond) UNE SEULE FOIS
			// au "vrai" premier Begin de la paire (pour éviter le multi-fixture)
			// ─────────────────────────────────────────────────────────────
			if ( firstEnter )
			{
				if ( m_projectileShuriken.count( attackerBody ) )
				{
					int& rebounds = m_shurikenReboundsLeft[attackerBody];
					if ( --rebounds < 0 )
						m_projectilesToDestroyMap[attackerBody] = ImGui::GetTime();
				}
				else if ( m_projectileArrows.count( attackerBody ) || m_projectileFireworks.count( attackerBody ) ||
						  m_projectileVampireKnives.count( attackerBody ) || m_projectileFrost.count( attackerBody ) ||
						  m_projectileExplosion.count( attackerBody ) || m_projectileElectricStaff.count( attackerBody ) ||
						  m_projectileFlask.count( attackerBody ) ) // <- AJOUT
				{
					m_projectilesToDestroyMap[attackerBody] = ImGui::GetTime();
				}
			}
		}

		// ─────────────────────────────────────────────────────────────────────────────
		// END TOUCH (sorties de sensors / shapes avec enableSensorEvents)
		// ─────────────────────────────────────────────────────────────────────────────
		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			const b2SensorEndTouchEvent& evt = sensorEvents.endEvents[i];
			if ( !b2Shape_IsValid( evt.sensorShapeId ) || !b2Shape_IsValid( evt.visitorShapeId ) )
				continue;

			b2ShapeId shapeA = evt.sensorShapeId;
			b2ShapeId shapeB = evt.visitorShapeId;

			b2BodyId bodyA = b2Shape_GetBody( shapeA );
			b2BodyId bodyB = b2Shape_GetBody( shapeB );
			if ( !BodyValid( bodyA ) || !BodyValid( bodyB ) )
				continue;

			b2Filter filterA = b2Shape_GetFilter( shapeA );
			b2Filter filterB = b2Shape_GetFilter( shapeB );

			const bool aIsSkin = ( filterA.categoryBits & CATEGORY_SKIN ) != 0;
			const bool bIsSkin = ( filterB.categoryBits & CATEGORY_SKIN ) != 0;
			const bool aIsWeaponOrProj = ( filterA.categoryBits & ( CATEGORY_WEAPON | CATEGORY_PROJECTILE ) ) != 0;
			const bool bIsWeaponOrProj = ( filterB.categoryBits & ( CATEGORY_WEAPON | CATEGORY_PROJECTILE ) ) != 0;

			if ( !( ( aIsSkin && bIsWeaponOrProj ) || ( bIsSkin && aIsWeaponOrProj ) ) )
				continue;

			b2BodyId attackerBody = aIsSkin ? bodyB : bodyA;

			// Résoudre la victime au niveau character
			const uint64_t victimSkinShapeId = b2StoreShapeId( aIsSkin ? shapeA : shapeB );
			auto itVictimMap = m_shapeToCharacter.find( victimSkinShapeId );
			if ( itVictimMap == m_shapeToCharacter.end() )
				continue;
			const uint64_t victimCharId = itVictimMap->second;

			const uint64_t attackerId = b2StoreBodyId( attackerBody );
			PairKey pk = MakePairKey( victimCharId, attackerId );

			if ( auto it = m_pairOverlap.find( pk ); it != m_pairOverlap.end() )
			{
				it->second = std::max( 0, it->second - 1 );
				if ( it->second == 0 )
					m_pairOverlap.erase( it ); // sortie complète -> on "réarme" la paire
			}
		}
	}

	void ProcessProjectileDestructions()
	{
		const double now = ImGui::GetTime();
		std::vector<b2BodyId> toDestroy;

		for ( auto it = m_projectilesToDestroyMap.begin(); it != m_projectilesToDestroyMap.end(); )
		{
			if ( now >= it->second )
			{
				toDestroy.push_back( it->first );
				it = m_projectilesToDestroyMap.erase( it );
			}
			else
				++it;
		}

		for ( b2BodyId id : toDestroy )
		{
			const bool wasExplosion = ( m_projectileExplosion.find( id ) != m_projectileExplosion.end() );
			if ( wasExplosion && BodyValid( id ) )
				TriggerExplosion( id );

			PurgeProjectile( id, /*destroyBody=*/true );
		}
	}

	void ProcessPendingCharacterDeaths()
	{
		if ( m_charactersToKill.empty() )
			return;

		std::sort( m_charactersToKill.begin(), m_charactersToKill.end(),
				   []( b2BodyId a, b2BodyId b ) { return b2StoreBodyId( a ) < b2StoreBodyId( b ); } );
		m_charactersToKill.erase(
			std::unique( m_charactersToKill.begin(), m_charactersToKill.end(),
						 []( b2BodyId a, b2BodyId b ) { return b2StoreBodyId( a ) == b2StoreBodyId( b ); } ),
			m_charactersToKill.end() );

		for ( b2BodyId body : m_charactersToKill )
			KillCharacterNow( body );

		m_charactersToKill.clear();
	}

	void AnimateWeaponPassives()
	{
		AnimateBoomerangs();
		AnimateExplosions();
		AnimateSlashLines();
		AnimateUnarmedGhostTrail();
		AnimateDeathPoofs();

		// --- update + render Leech Rays ---
		double now = ImGui::GetTime();
		float dt = ( m_lastLeechUpdateTime == 0.0 ) ? 0.f : float( now - m_lastLeechUpdateTime );
		m_lastLeechUpdateTime = now;

		UpdateLeechRays( dt );
		RenderLeechRays();
		// -----------------------------------

		for ( const std::string& name : kAllCharacters )
		{
			b2BodyId charId = GetCharacterIdByName( name );
			if ( !BodyValid( charId ) )
				continue;

			uint64_t id = b2StoreBodyId( charId );
			bool blink = false;

			auto it = m_lastHitBlinkTime.find( id );
			if ( it != m_lastHitBlinkTime.end() && now - it->second < kHitBlinkDuration )
				blink = true;

			float blinkAmount = 0.0f;
			{
				auto itBlink = m_lastHitBlinkTime.find( id );
				if ( itBlink != m_lastHitBlinkTime.end() )
				{
					double dt = now - itBlink->second;
					if ( dt < kHitBlinkDuration )
					{
						blinkAmount = 1.0f - float( dt / kHitBlinkDuration );
					}
				}
			}

			bool poisoned = ( m_poisonBuildUp.find( id ) != m_poisonBuildUp.end() );
			float poisonPulse = 0.0f;
			if ( poisoned )
				poisonPulse = 0.35f + 0.65f * ( 0.5f + 0.5f * std::sin( float( now ) * 4.5f ) );

			bool slashed = ( m_slashBuildUp.find( id ) != m_slashBuildUp.end() );
			float slashPulse = 0.0f;
			if ( slashed )
				slashPulse = 0.25f + 0.75f * ( 0.5f + 0.5f * std::sin( float( now ) * 9.0f ) );

			ApplyCharacterVisual( charId, blinkAmount, poisoned, poisonPulse, slashed, slashPulse );
		}
	}

	void AnimateBoomerangs()
	{
		// on n’utilise pas m_boomerangId / m_boomerangJointId ici
		for ( auto& [wid, st] : m_boomerangs )
		{
			// joint correspondant : pris via le mapping générique
			auto itJ = m_weaponToJoint.find( wid );
			if ( itJ == m_weaponToJoint.end() || !JointValid( itJ->second ) )
				continue;

			const b2JointId j = itJ->second;

			// owner du boomerang (pour le test de freeze côté perso)
			b2BodyId owner = b2_nullBodyId;
			if ( auto itO = m_weaponOwner.find( wid ); itO != m_weaponOwner.end() )
				owner = itO->second;

			// dt “freeze-friendly”
			const double now = ImGui::GetTime();
			const double dt = ( st.lastTime == 0.0 ) ? 0.0 : ( now - st.lastTime );
			st.lastTime = now;

			const bool frozen = ( BodyValid( owner ) && IsBodyCurrentlyFrozen( owner ) ) ||
								( BodyValid( st.body ) && IsBodyCurrentlyFrozen( st.body ) );
			if ( !frozen )
				st.phase += dt * double( st.animSpeed );

			const float t = float( st.phase );

			// Rosace : r = A * cos(k*t), 5 branches
			const float A = st.reachA;
			const float k = 5.0f;
			const float r = A * std::cos( k * t );

			// écrasement vertical proportionnel à B/A
			const float scaleY = ( st.reachA > 1e-4f ) ? ( st.reachB / st.reachA ) : 1.0f;

			b2Vec2 offset = { r * std::cos( t ), scaleY * r * std::sin( t ) };

			// orientation tangentielle
			const float rp = -A * k * std::sin( k * t );
			const float dx = rp * std::cos( t ) - r * std::sin( t );
			const float dy = rp * std::sin( t ) + r * std::cos( t );
			const float ang = std::atan2( dy, dx );

			b2Transform tf = { offset, b2MakeRot( ang ) };
			b2Joint_SetLocalFrameA( j, tf );
		}
	}

	void AnimateExplosions()
	{
		const double now = ImGui::GetTime();
		ImDrawList* dl = ImGui::GetForegroundDrawList();

		std::vector<ExplosionAnim> still;
		still.reserve( m_activeExplosions.size() );

		// Monde -> pixels
		auto worldToPixels = [&]( const b2Vec2& world, float worldRadius ) -> std::pair<ImVec2, float> {
			b2Vec2 s0 = ConvertWorldToScreen( &m_context->camera, world );
			b2Vec2 s1 = ConvertWorldToScreen( &m_context->camera, { world.x + worldRadius, world.y } );
			return { ImVec2( s0.x, s0.y ), std::abs( s1.x - s0.x ) };
		};

		// rgb 0xRRGGBB -> ImU32
		auto RGBtoImU32 = []( uint32_t rgb, int a ) -> ImU32 {
			return IM_COL32( ( rgb >> 16 ) & 255, ( rgb >> 8 ) & 255, rgb & 255, std::clamp( a, 0, 255 ) );
		};

		// Couleur de base à partir du perso owner (fallback chaud si introuvable)
		auto GetOwnerBaseRGB = [&]( uint64_t ownerStoredId ) -> uint32_t {
			if ( ownerStoredId == 0 )
				return 0xFFB74D;
			b2BodyId ch = GetCharacterBodyByStoredId( ownerStoredId );
			if ( !BodyValid( ch ) )
				return 0xFFB74D;

			const uint64_t cKey = b2StoreBodyId( ch );

			// 1) si on connaît la "skin shape", on prend sa couleur mémorisée
			if ( auto itSkin = m_characterSkinShape.find( cKey );
				 itSkin != m_characterSkinShape.end() && b2Shape_IsValid( itSkin->second ) )
			{
				uint64_t skey = b2StoreShapeId( itSkin->second );
				if ( auto itC = m_shapeBaseColor.find( skey ); itC != m_shapeBaseColor.end() )
					return itC->second;
			}

			// 2) sinon on cherche une forme qui "appartient" au perso et dont on a la couleur
			b2ShapeId shapes[128];
			int n = b2Body_GetShapes( ch, shapes, 128 );
			for ( int i = 0; i < n; ++i )
			{
				uint64_t skey = b2StoreShapeId( shapes[i] );
				if ( auto itBel = m_shapeToCharacter.find( skey ); itBel != m_shapeToCharacter.end() && itBel->second == cKey )
				{
					if ( auto itC = m_shapeBaseColor.find( skey ); itC != m_shapeBaseColor.end() )
						return itC->second;
				}
			}
			return 0xFFB74D;
		};

		// mêmes paramètres que ta version originale (on garde le “feeling”)
		const float kDur = 0.60f;
		const int kRings = 3;
		const float kRingSpread = 0.22f;
		const float kMaxOvershoot = 1.35f;

		for ( const ExplosionAnim& e : m_activeExplosions )
		{
			float t = float( now - e.startTime );
			if ( t >= kDur )
				continue;

			float u = t / kDur; // [0..1]
			auto [p, oneUnitPx] = worldToPixels( e.pos, 1.0f );

			// Palette dérivée du perso
			uint32_t baseRGB = GetOwnerBaseRGB( e.ownerCharacterId );
			uint32_t lightRGB = LerpColor( baseRGB, 0xFFFFFF, 0.35f );
			uint32_t glowRGB = LerpColor( baseRGB, 0xFFFFFF, 0.65f );
			uint32_t darkRGB = LerpColor( baseRGB, 0x000000, 0.25f );

			// (1) petit flash doux
			{
				float flashU = 1.0f - u;
				float flashRpx = oneUnitPx * e.radius * ( 0.22f * flashU );
				int a = int( 190.f * flashU );
				if ( flashRpx > 0.5f && a > 0 )
					dl->AddCircleFilled( p, flashRpx, RGBtoImU32( glowRGB, a ) );
			}

			// (2) remplissage très léger = zone de dégâts (même rayon que la physique)
			{
				float rpx = oneUnitPx * e.radius;
				int a = int( 80.f * ( 1.0f - std::min( u, 0.8f ) ) ); // quasi stable puis fade
				dl->AddCircleFilled( p, rpx, RGBtoImU32( lightRGB, a ) );
			}

			// (3) ANNEAU NET = radius de dégâts exact
			{
				float rpx = oneUnitPx * e.radius;
				float keep = ( u < 0.80f ) ? 1.0f : std::max( 0.f, 1.0f - ( u - 0.80f ) / 0.20f );
				int a = int( 255 * keep );
				const float thick = 3.0f; // épaisseur en pixels (lisible)
				dl->AddCircle( p, rpx, RGBtoImU32( baseRGB, a ), 96, thick );
				// liseré interne pour le contraste
				dl->AddCircle( p, std::max( 0.0f, rpx - 1.0f ), RGBtoImU32( darkRGB, int( 120 * keep ) ), 96, 1.0f );
			}

			// (4) tes shockwaves, mais teintées avec la palette du perso
			for ( int i = 0; i < kRings; ++i )
			{
				float start = i * kRingSpread;
				float ringU = ( u - start ) / ( 1.0f - start );
				if ( ringU <= 0.0f )
					continue;
				ringU = std::min( ringU, 1.0f );
				float eased = 1.0f - ( 1.0f - ringU ) * ( 1.0f - ringU );

				float ringWorldR = e.radius * ( 0.50f + eased * kMaxOvershoot );
				float ringPx = oneUnitPx * ringWorldR;
				float thick = std::max( 1.0f, 6.0f * ( 1.0f - ringU ) );
				int a = int( 180.0f * ( 1.0f - ringU ) );

				dl->AddCircle( p, ringPx, RGBtoImU32( glowRGB, a ), 96, thick );
			}

			still.push_back( e );
		}

		m_activeExplosions.swap( still );
	}

	void AnimateSlashLines()
	{
		if ( !m_context )
			return;

		const double now = ImGui::GetTime();
		ImDrawList* dl = ImGui::GetForegroundDrawList();

		std::vector<SlashLine> keep;
		keep.reserve( m_activeSlashLines.size() );

		auto W2S = [&]( const b2Vec2& w ) -> ImVec2 {
			b2Vec2 s = ConvertWorldToScreen( &m_context->camera, w );
			return ImVec2( s.x, s.y );
		};

		for ( const SlashLine& L : m_activeSlashLines )
		{
			float u = float( ( now - L.start ) / L.dur );
			if ( u >= 1.0f )
				continue;

			float fade = 1.0f - u;

			ImVec2 A = W2S( L.A );
			ImVec2 B = W2S( L.B );

			float dx = B.x - A.x, dy = B.y - A.y;
			float pxLen = std::sqrt( dx * dx + dy * dy );

			float baseThick = std::clamp( pxLen * 0.035f, 1.5f, 7.0f );
			float thGlow = baseThick * ( 0.90f + 0.60f * fade );
			float thCore = std::max( 1.0f, baseThick * 0.45f );
			int aGlow = int( 180.0f * fade );
			int aCore = int( 255.0f * std::min( 1.0f, 0.6f + 0.4f * fade ) );

			// --- wobble (respiration subtile) ---
			// amplitude et fréquence (ajuste à ton goût)
			constexpr float kWobbleAmp = 0.10f;	 // ±10%
			constexpr float kWobbleFreq = 25.0f; // Hz en temps ImGui
			float wobble = 1.0f + kWobbleAmp * std::sin( float( now ) * kWobbleFreq );
			thGlow *= wobble;
			thCore *= wobble;
			// ------------------------------------

			ImU32 glow = IM_COL32( 0, 200, 255, aGlow );
			ImU32 core = IM_COL32( 255, 255, 255, aCore );

			dl->AddLine( A, B, glow, thGlow ); // halo
			dl->AddLine( A, B, core, thCore ); // cœur

			float capR = std::max( 1.0f, thCore * 0.60f );
			dl->AddCircleFilled( A, capR, core );
			dl->AddCircleFilled( B, capR, core );

			keep.push_back( L );
		}
		m_activeSlashLines.swap( keep );
	}

	void AnimateUnarmedGhostTrail()
	{
		if ( !BodyValid( m_characterIdUnarmed ) || !m_context )
			return;

		const double now = ImGui::GetTime();
		const b2Vec2 p = b2Body_GetPosition( m_characterIdUnarmed );

		// --- spawn/TTL inchangé ---
		if ( !m_unarmedGhostInit )
		{
			m_unarmedLastGhostPos = p;
			m_unarmedGhostInit = true;
		}
		if ( b2Distance( p, m_unarmedLastGhostPos ) >= m_unarmedGhostMinDist )
		{
			if ( (int)m_unarmedGhosts.size() >= m_unarmedGhostMax )
				m_unarmedGhosts.pop_front();
			m_unarmedGhosts.push_back( GhostCircle{ p, now + m_unarmedGhostTTL } );
			m_unarmedLastGhostPos = p;
		}
		while ( !m_unarmedGhosts.empty() && m_unarmedGhosts.front().death <= now )
			m_unarmedGhosts.pop_front();

		// --- dessin : fin, fade rapide, fill doux ---
		ImDrawList* dl = ImGui::GetForegroundDrawList();
		auto worldToPixels = [&]( const b2Vec2& center, float worldRadius ) -> std::pair<ImVec2, float> {
			b2Vec2 s0 = ConvertWorldToScreen( &m_context->camera, center );
			b2Vec2 s1 = ConvertWorldToScreen( &m_context->camera, { center.x + worldRadius, center.y } );
			return { ImVec2( s0.x, s0.y ), std::abs( s1.x - s0.x ) };
		};

		int idx = 0, N = (int)m_unarmedGhosts.size();
		for ( const GhostCircle& g : m_unarmedGhosts )
		{
			// fraction de vie restante
			float life01 =
				( m_unarmedGhostTTL > 0.0 ) ? std::clamp( float( ( g.death - now ) / m_unarmedGhostTTL ), 0.0f, 1.0f ) : 0.0f;

			// fade plus rapide au début (gamma > 1)
			float alpha01 = std::pow( life01, 2.2f );

			// plus ancien → un peu plus transparent
			if ( N > 1 )
			{
				float along = float( idx ) / float( N - 1 ); // 0 (ancien) → 1 (récent)
				alpha01 *= 0.6f + 0.4f * along;				 // anciens = 60% du récent
			}

			auto [sp, rpx] = worldToPixels( g.p, m_unarmedGhostRadius );
			const float strokePx = 1.0f; // trait fin ~1 px
			const int aStroke = int( 255.0f * alpha01 );
			const int aFill = int( 255.0f * alpha01 * 0.12f ); // très léger remplissage

			if ( aFill > 0 )
				dl->AddCircleFilled( sp, rpx, IM_COL32( 0, 0, 0, aFill ), 64 );
			if ( aStroke > 0 )
				dl->AddCircle( sp, rpx, IM_COL32( 0, 0, 0, aStroke ), 64, strokePx );

			++idx;
		}
	}

	// --- utils couleur & easing ---
	static inline float Smooth01( float u )
	{
		u = std::clamp( u, 0.f, 1.f );
		return u * u * ( 3.f - 2.f * u );
	}
	static inline ImU32 RGB_A( uint32_t rgb, int a )
	{
		return IM_COL32( ( rgb >> 16 ) & 255, ( rgb >> 8 ) & 255, rgb & 255, std::clamp( a, 0, 255 ) );
	}

	// Couleur principale d’un perso (fallback si introuvable)
	uint32_t GetCharacterMainColor( b2BodyId characterId )
	{
		if ( !BodyValid( characterId ) )
			return 0xFF6EC1; // bleu doux
		const uint64_t cKey = b2StoreBodyId( characterId );

		// 1) skin shape connue ?
		auto itSkin = m_characterSkinShape.find( cKey );
		if ( itSkin != m_characterSkinShape.end() && b2Shape_IsValid( itSkin->second ) )
		{
			uint64_t skey = b2StoreShapeId( itSkin->second );
			auto itCol = m_shapeBaseColor.find( skey );
			if ( itCol != m_shapeBaseColor.end() )
				return itCol->second;
		}

		// 2) sinon, prend la première forme appartenant au perso dont on a la couleur
		b2ShapeId shapes[64];
		int n = b2Body_GetShapes( characterId, shapes, 64 );
		for ( int i = 0; i < n; ++i )
		{
			uint64_t skey = b2StoreShapeId( shapes[i] );
			auto itBel = m_shapeToCharacter.find( skey );
			if ( itBel != m_shapeToCharacter.end() && itBel->second == cKey )
			{
				auto itCol = m_shapeBaseColor.find( skey );
				if ( itCol != m_shapeBaseColor.end() )
					return itCol->second;
			}
		}
		return 0xFF6EC1;
	}

	// Spawner : un rayon qui part du point d’impact vers le joueur (owner)
	void SpawnLeechRay( const b2Vec2& hitPointWorld, b2BodyId owner )
	{
		LeechRay r;
		r.fromWorld = hitPointWorld;
		r.toBody = owner;
		r.t = 0.f;
		r.T = 0.20f; // ← 0.1 s au lieu de 1.0f
		r.w0 = 2.0f * kPixelSize;
		r.w1 = 0.4f * kPixelSize;
		r.rgba = GetCharacterMainColor( owner );
		m_leechRays.push_back( r );
	}

	// Update temps-vie (dt en secondes)
	void UpdateLeechRays( float dt )
	{
		for ( size_t i = 0; i < m_leechRays.size(); )
		{
			LeechRay& r = m_leechRays[i];
			r.t += dt;
			if ( r.t >= r.T )
			{
				m_leechRays[i] = m_leechRays.back();
				m_leechRays.pop_back();
			}
			else
			{
				++i;
			}
		}
	}

	// Dessin en overlay (ImGui) – épaisseur décroit, alpha fade
	void RenderLeechRays()
	{
		if ( !m_context )
			return;
		ImDrawList* dl = ImGui::GetForegroundDrawList();

		auto W2S = [&]( const b2Vec2& w ) -> ImVec2 {
			const b2Vec2 s = ConvertWorldToScreen( &m_context->camera, w );
			return ImVec2( s.x, s.y );
		};

		auto WorldToPx = [&]( float worldScalar ) -> float {
			const b2Vec2 s0 = ConvertWorldToScreen( &m_context->camera, { 0.f, 0.f } );
			const b2Vec2 s1 = ConvertWorldToScreen( &m_context->camera, { worldScalar, 0.f } );
			return std::fabs( s1.x - s0.x );
		};

		const float wobbleAmp = 0.06f;	// légère respiration d’épaisseur
		const float wobbleFreq = 18.0f; // Hz (temps ImGui)

		double now = ImGui::GetTime();

		for ( const LeechRay& r : m_leechRays )
		{
			b2Vec2 A = r.fromWorld;
			b2Vec2 B = B2_IS_NON_NULL( r.toBody ) ? b2Body_GetPosition( r.toBody ) : r.fromWorld;

			float u = Smooth01( r.t / std::max( 1e-6f, r.T ) ); // 0..1
			float w = r.w0 + ( r.w1 - r.w0 ) * u;

			// wobble
			w *= ( 1.0f + wobbleAmp * std::sin( float( now ) * wobbleFreq ) );

			// conversion en pixels pour l’épaisseur ImGui
			float wpx = std::max( 1.0f, WorldToPx( w ) );
			float wpxGlow = std::max( 1.0f, wpx * ( 1.8f - 0.6f * u ) ); // glow + large au début
			float wpxCore = std::max( 1.0f, wpx * 0.55f );

			int aGlow = int( 200.0f * ( 1.0f - u ) );
			int aCore = int( 255.0f * ( 0.65f + 0.35f * ( 1.0f - u ) ) );

			ImVec2 SA = W2S( A ), SB = W2S( B );

			// halo (couleur perso un peu plus claire)
			uint32_t glowRGB = LerpColor( r.rgba, 0xFFFFFF, 0.35f );
			dl->AddLine( SA, SB, RGB_A( glowRGB, aGlow ), wpxGlow );

			// cœur
			dl->AddLine( SA, SB, RGB_A( r.rgba, aCore ), wpxCore );

			// petits caps ronds
			float capR = std::max( 1.0f, wpxCore * 0.55f );
			dl->AddCircleFilled( SA, capR, RGB_A( r.rgba, aCore ) );
			dl->AddCircleFilled( SB, capR, RGB_A( r.rgba, aCore ) );
		}
	}

	void SpawnDeathPoof( b2BodyId characterBody )
	{
		if ( !BodyValid( characterBody ) || !m_context )
			return;

		const b2Vec2 p0 = b2Body_GetPosition( characterBody );
		const uint32_t base = GetCharacterMainColor( characterBody );
		const uint32_t edge = LerpColor( base, 0x000000, 0.22f ); // bord plus foncé
		const double now = ImGui::GetTime();

		// Distributions : 5 cercles, tailles et vitesses variées
		std::uniform_real_distribution<float> rdist( 0.25f * kCharacterRadius, 0.55f * kCharacterRadius );
		std::uniform_real_distribution<float> spd( 1.0f, 5.0f ); // m/s -> "quelques mètres" sur ~1s
		std::uniform_real_distribution<float> ang( 0.0f, 2.0f * b2_pi );
		std::uniform_real_distribution<float> damp( 1.5f, 4.0f );  // amortissement linéaire s^-1
		std::uniform_real_distribution<float> life( 0.85f, 1.0f ); // durée de vie (s)

		const int N = 5;
		for ( int i = 0; i < N; ++i )
		{
			float a = ang( m_rngAll );
			float s = spd( m_rngAll );
			float rad = rdist( m_rngAll );

			DeathPoofBall d{};
			d.p = p0;
			d.v = { s * std::cos( a ), s * std::sin( a ) };
			d.r = rad;
			d.damping = damp( m_rngAll );
			d.fillRGB = base;
			d.edgeRGB = edge;
			d.t0 = now;
			d.t1 = now + life( m_rngAll );
			d.last = now;

			m_deathPoofBalls.push_back( d );
		}
	}

	void AnimateDeathPoofs()
	{
		if ( !m_context )
			return;

		ImDrawList* dl = ImGui::GetForegroundDrawList();
		const double now = ImGui::GetTime();

		auto W2S = [&]( const b2Vec2& w ) -> ImVec2 {
			const b2Vec2 s = ConvertWorldToScreen( &m_context->camera, w );
			return ImVec2( s.x, s.y );
		};

		auto WorldToPx = [&]( float worldScalar ) -> float {
			const b2Vec2 s0 = ConvertWorldToScreen( &m_context->camera, { 0.f, 0.f } );
			const b2Vec2 s1 = ConvertWorldToScreen( &m_context->camera, { worldScalar, 0.f } );
			return std::fabs( s1.x - s0.x );
		};
		auto RGB_A = [&]( uint32_t rgb, int a ) -> ImU32 {
			return IM_COL32( ( rgb >> 16 ) & 255, ( rgb >> 8 ) & 255, rgb & 255, std::clamp( a, 0, 255 ) );
		};

		std::vector<DeathPoofBall> keep;
		keep.reserve( m_deathPoofBalls.size() );

		for ( DeathPoofBall& d : m_deathPoofBalls )
		{
			const double dt = ( d.last == 0.0 ) ? 0.0 : ( now - d.last );
			d.last = now;

			// Damping linéaire sur la vitesse + intégration position
			float k = std::exp( -d.damping * float( dt ) );
			d.v *= k;
			d.p += d.v * float( dt );

			// Vie / fade
			float u = float( ( now - d.t0 ) / std::max( 1e-6, d.t1 - d.t0 ) );
			u = std::clamp( u, 0.f, 1.f );
			float fade = 1.0f - u;

			if ( u < 1.0f && fade > 0.0f )
			{
				ImVec2 sp = W2S( d.p );
				float rp = WorldToPx( d.r );

				// Remplissage = couleur joueur (un peu transparent)
				int aFill = int( 210.0f * fade );
				if ( aFill > 0 && rp > 0.5f )
					dl->AddCircleFilled( sp, rp, RGB_A( d.fillRGB, aFill ), 64 );

				// Bord = un peu plus foncé, trait net
				int aEdge = int( 255.0f * fade );
				float edgePx = std::max( 1.5f, std::min( 3.0f, rp * 0.10f ) ); // 1.5–3 px env.
				dl->AddCircle( sp, rp, RGB_A( d.edgeRGB, aEdge ), 64, edgePx );

				keep.push_back( d );
			}
		}

		m_deathPoofBalls.swap( keep );
	}

	void UpdatePoison()
	{
		const double now = ImGui::GetTime();
		std::unordered_set<uint64_t> toRemove;

		for ( auto& [victimId, state] : m_poisonBuildUp )
		{
			double& lastTick = state.first;
			int& ticksLeft = state.second;

			if ( ticksLeft > 0 && now - lastTick >= 1.0 )
			{
				auto itHp = m_characterHP.find( victimId );
				if ( itHp != m_characterHP.end() )
				{
					int& hp = itHp->second;
					if ( hp > 0 )
					{
						hp = std::max( 0, hp - 2 );
						--ticksLeft;
						lastTick = now;
						m_lastHitBlinkTime[victimId] = now;
						if ( hp == 0 )
						{
							b2BodyId victimBody = GetCharacterBodyByStoredId( victimId );
							if ( BodyValid( victimBody ) )
								RequestKillCharacter( victimBody );
							ticksLeft = 0;
							toRemove.insert( victimId );
						}
					}
					else
					{
						toRemove.insert( victimId );
					}
				}
				else
				{
					toRemove.insert( victimId );
				}
			}
			if ( ticksLeft <= 0 )
				toRemove.insert( victimId );
		}
		for ( uint64_t id : toRemove )
			m_poisonBuildUp.erase( id );
	}

	void UpdateSlashes()
	{
		const double now = ImGui::GetTime();
		std::vector<uint64_t> toErase;

		for ( auto& kv : m_slashBuildUp )
		{
			const uint64_t victimId = kv.first;
			double& lastTick = kv.second.first; // dernier tick effectué
			int& ticksLeft = kv.second.second;	// nb de ticks restants (1 dmg / tick)

			if ( ticksLeft <= 0 )
			{
				toErase.push_back( victimId );
				continue;
			}

			// cadence : toutes les m_slashTickInterval secondes
			if ( now - lastTick < m_slashTickInterval )
				continue;

			lastTick = now; // on valide le tick courant

			auto itHp = m_characterHP.find( victimId );
			if ( itHp != m_characterHP.end() && itHp->second > 0 )
			{
				// 1 dégât par tick, pas plus
				itHp->second = std::max( 0, itHp->second - 1 );

				// FX & son
				b2BodyId victimBody = GetCharacterBodyByStoredId( victimId );
				b2Vec2 pos = BodyValid( victimBody ) ? b2Body_GetPosition( victimBody ) : b2Vec2{ 0.f, 0.f };
				m_lastHitBlinkTime[victimId] = now;
				PlayHitEventSound( pos, /*speed=*/1.0f );
				if ( BodyValid( victimBody ) )
					SpawnSlashLines( victimBody, 1 ); // 1 trait visuel par tick

				if ( itHp->second == 0 && BodyValid( victimBody ) )
					RequestKillCharacter( victimBody );
			}

			// on consomme un tick
			--ticksLeft;
			if ( ticksLeft <= 0 )
				toErase.push_back( victimId );
		}

		for ( uint64_t id : toErase )
			m_slashBuildUp.erase( id );
	}

	void UpdateTurrets( double now )
	{
		const double turretFireInterval = 0.2;

		for ( auto it = m_turretIds.begin(); it != m_turretIds.end(); )
		{
			b2BodyId turretId = *it;

			if ( !BodyValid( turretId ) )
			{
				it = m_turretIds.erase( it );
				m_turretLastShot.erase( turretId );
				m_turretOwner.erase( turretId );
				continue;
			}

			double& last = m_turretLastShot[turretId];
			if ( now - last > turretFireInterval )
			{
				FireTurretProjectile( turretId );
				last = now;
			}

			++it;
		}
	}

	// ========= Utilitaires gameplay (extrait) =========

	void PlayFromBankSafe( int bank )
	{
		int b = bank;
		if ( b < 0 || m_audioManager.GetBankSoundCount( b ) == 0 )
		{
			b = ( m_banks.hits >= 0 && m_audioManager.GetBankSoundCount( m_banks.hits ) > 0 ) ? m_banks.hits : m_banks.rebound;
		}
		if ( b >= 0 && m_audioManager.GetBankSoundCount( b ) > 0 )
		{
			m_audioManager.SetCurrentBank( b );
			m_audioManager.PlayRandomInBank();
		}
	}

	void InitAudioBanks()
	{
		auto addBank = [&]( bool first, const char* path ) -> int {
			size_t before = m_audioManager.GetBankCount();
			if ( first )
				m_audioManager.LoadFromDirectory( path );
			else
				m_audioManager.AddFromDirectory( path );
			int idx = (int)m_audioManager.GetBankCount() - 1;
			int n = (int)m_audioManager.GetBankSoundCount( idx );
			std::cout << "[Audio] " << ( first ? "LOAD" : "ADD " ) << " -> bank " << idx << " <- " << path << " | sounds=" << n
					  << "\n";
			return idx;
		};

		const char* root = "D:/Sound & Fx/audio/weaponballs";

		struct Spec
		{
			const char* sub;
			int* out;
			bool first = false;
		};
		Spec specs[] = {
			{ "hits/hit_rebound", &m_banks.rebound, true },
			{ "hits", &m_banks.hits },
			{ "projectiles/projectile_bow", &m_banks.proj_bow },
			{ "projectiles/projectile_crossbow", &m_banks.proj_crossbow },
			{ "projectiles/projectile_vampire_knives", &m_banks.proj_vampire },
			{ "projectiles/projectile_shuriken", &m_banks.proj_shuriken },
			{ "projectiles/projectile_frost_staff", &m_banks.proj_frost },
			{ "projectiles/projectile_explosion_staff", &m_banks.proj_explosion },
			{ "projectiles/projectile_electric_staff", &m_banks.proj_electric_staff },
			{ "projectiles/projectile_poison_blowgun", &m_banks.proj_poison },
			{ "projectiles/projectile_turret", &m_banks.proj_turret },
			{ "projectiles/projectile_flask", &m_banks.proj_flask },

			{ "projectiles/projectile_impact", &m_banks.proj_impact },

			{ "hits/hit_melee", &m_banks.melee_hit },
			{ "hits/hit_parry", &m_banks.motor_invert },
			{ "explosions", &m_banks.explosion_blast },

			{ "hits/hit_bow", &m_banks.w_bow },
			{ "hits/hit_crossbow", &m_banks.w_crossbow },
			{ "hits/hit_sword", &m_banks.w_sword },
			{ "hits/hit_axe", &m_banks.w_axe },
			{ "hits/hit_dagger", &m_banks.w_dagger },
			{ "hits/hit_katana", &m_banks.w_katana },
			{ "hits/hit_trident", &m_banks.w_trident },
			{ "hits/hit_hammer", &m_banks.w_hammer },
			{ "hits/hit_poison_blowgun", &m_banks.w_poison_blowgun },
			{ "hits/hit_club", &m_banks.w_club },
			{ "hits/hit_scythe", &m_banks.w_scythe },
			{ "hits/hit_spear", &m_banks.w_spear },
			{ "hits/hit_kunai", &m_banks.w_kunai },
			{ "hits/hit_boomerang", &m_banks.w_boomerang },
			{ "hits/hit_shuriken", &m_banks.w_shuriken },
			{ "hits/hit_big_sword", &m_banks.w_big_sword },
			{ "hits/hit_electric_staff", &m_banks.w_electric_staff },
			{ "hits/hit_explosion_staff", &m_banks.w_explosion_staff },
			{ "hits/hit_frost_staff", &m_banks.w_frost_staff },
			{ "hits/hit_mahoraga", &m_banks.w_mahoraga },
			{ "hits/hit_vampire_knife", &m_banks.w_vampire_knife },
			{ "hits/hit_shield", &m_banks.w_shield },
			{ "hits/hit_wrench", &m_banks.w_wrench },
			{ "hits/hit_unarmed", &m_banks.w_unarmed },

			// --- Nouveaux (pixel_art) ---
			{ "hits/hit_glaive", &m_banks.w_glaive },
			{ "hits/hit_sickle_r", &m_banks.w_sickle_r },
			{ "hits/hit_flask", &m_banks.w_flask },
			{ "hits/hit_lance", &m_banks.w_lance },

		};

		for ( auto& s : specs )
		{
			std::string p = std::string( root ) + "/" + s.sub;
			*s.out = addBank( s.first, p.c_str() );
		}

		const size_t nb = m_audioManager.GetBankCount();
		std::cout << "[Audio] total banks=" << nb << "\n";
		for ( size_t b = 0; b < nb; ++b )
			std::cout << "  bank " << b << " -> " << m_audioManager.GetBankSoundCount( b ) << " sounds\n";
	}

	void DumpAudioBanks()
	{
		const size_t nb = m_audioManager.GetBankCount();
		std::cout << "[Audio] banques=" << nb << "\n";
		for ( size_t b = 0; b < nb; ++b )
			std::cout << "  bank " << b << " -> " << m_audioManager.GetBankSoundCount( b ) << " sons\n";
	}

	void PlayReboundSound( const b2Vec2& pos, float speed )
	{
		int bank = ( m_banks.rebound >= 0 ) ? m_banks.rebound : m_banks.hits;
		if ( bank >= 0 && m_audioManager.GetBankSoundCount( bank ) > 0 )
		{
			m_audioManager.SetCurrentBank( bank );
			m_audioManager.HandleHitEffect( pos, speed, m_stepCounter );
		}
	}

	void PlayMotorInvertSound( const b2Vec2& )
	{
		PlayFromBankSafe( m_banks.motor_invert );
	}

	void PlayHitEventSound( const b2Vec2& pos, float speed )
	{
		int bank = ( m_banks.hits >= 0 ) ? m_banks.hits : m_banks.rebound;
		if ( bank >= 0 && m_audioManager.GetBankSoundCount( bank ) > 0 )
		{
			m_audioManager.SetCurrentBank( bank );
			m_audioManager.HandleHitEffect( pos, std::max( 0.1f, speed ), m_stepCounter );
		}
	}

	// [AJOUT] — son unique quand un projectile blesse un personnage
	void PlayProjectileImpactSound( const b2Vec2& pos, float speed )
	{
		int bank = m_banks.proj_impact;

		// Fallback robuste si la banque n’est pas chargée/vide
		if ( bank < 0 || m_audioManager.GetBankSoundCount( bank ) == 0 )
		{
			bank = ( m_banks.hits >= 0 && m_audioManager.GetBankSoundCount( m_banks.hits ) > 0 ) ? m_banks.hits : m_banks.rebound;
		}

		if ( bank >= 0 && m_audioManager.GetBankSoundCount( bank ) > 0 )
		{
			m_audioManager.SetCurrentBank( bank );
			m_audioManager.HandleHitEffect( pos, std::max( 0.1f, speed ), m_stepCounter );
		}
	}

	void PlayExplosionBlast( const b2Vec2& pos, float power )
	{
		int bank = ( m_banks.explosion_blast >= 0 && m_audioManager.GetBankSoundCount( m_banks.explosion_blast ) > 0 )
					   ? m_banks.explosion_blast
					   : m_banks.proj_explosion; // fallback si la banque dédiée est vide

		if ( bank >= 0 && m_audioManager.GetBankSoundCount( bank ) > 0 )
		{
			m_audioManager.SetCurrentBank( bank );
			// On réutilise HandleHitEffect pour la spatialisation/variation.
			// 'power' pilote l’intensité (volume/pitch interne à AudioManager).
			m_audioManager.HandleHitEffect( pos, std::max( 0.2f, power ), m_stepCounter );
		}
	}

	void PlayMeleeDamageSound( b2BodyId weapon, const b2Vec2& /*pos*/ )
	{
		int bank = FindMeleeBankFor( weapon );
		if ( weapon == m_characterIdUnarmed || weapon == b2_nullBodyId )
			bank = m_banks.w_unarmed;
		if ( bank < 0 )
			bank = m_banks.melee_hit;
		PlayFromBankSafe( bank );
	}

	void PlayProjectileLaunchSound( ProjectileKind kind, const b2Vec2& )
	{
		int bank = m_banks.hits;
		switch ( kind )
		{
			case ProjectileKind::Arrow:
				bank = m_banks.proj_bow;
				break;
			case ProjectileKind::Firework:
				bank = m_banks.proj_crossbow;
				break;
			case ProjectileKind::VampireKnife:
				bank = m_banks.proj_vampire;
				break;
			case ProjectileKind::Shuriken:
				bank = m_banks.proj_shuriken;
				break;
			case ProjectileKind::Frost:
				bank = m_banks.proj_frost;
				break;
			case ProjectileKind::Explosion:
				bank = m_banks.proj_explosion;
				break;
			case ProjectileKind::Electric:
				bank = m_banks.proj_electric_staff;
				break;
			case ProjectileKind::PoisonDart:
				bank = m_banks.proj_poison;
				break;
			case ProjectileKind::Turret:
				bank = m_banks.proj_turret;
				break;
			// NEW
			case ProjectileKind::Flask:
				bank = m_banks.proj_flask;
				break;
		}
		PlayFromBankSafe( bank );
	}

	void RegisterProjectile( b2BodyId proj, ProjectileKind kind, b2BodyId owner )
	{
		if ( !B2_IS_NON_NULL( proj ) )
			return;

		switch ( kind )
		{
			case ProjectileKind::Arrow:
				m_projectileArrows.insert( proj );
				break;
			case ProjectileKind::Firework:
				m_projectileFireworks.insert( proj );
				break;
			case ProjectileKind::VampireKnife:
				m_projectileVampireKnives.insert( proj );
				break;
			case ProjectileKind::Shuriken:
				m_projectileShuriken.insert( proj );
				m_shurikenReboundsLeft[proj] = 1 + m_shurikenBonusRebounds;
				break;
			case ProjectileKind::Frost:
				m_projectileFrost.insert( proj );
				break;
			case ProjectileKind::Explosion:
				m_projectileExplosion.insert( proj );
				m_weaponDamage[b2StoreBodyId( proj )] = 0;
				break;
			case ProjectileKind::Electric:
				m_projectileElectricStaff.insert( proj );
				break;
			case ProjectileKind::PoisonDart:
				m_projectilePoisonDarts.insert( proj );
				break;
			case ProjectileKind::Turret:
				m_projectileTurrets.insert( proj );
				break;
			// NEW
			case ProjectileKind::Flask:
				m_projectileFlask.insert( proj );
				break;
		}

		m_weaponOwner[b2StoreBodyId( proj )] = owner;

		if ( BodyValid( proj ) )
			m_projectileSpawnPos[proj] = b2Body_GetPosition( proj );
		else
			m_projectileSpawnPos.erase( proj );

		if ( m_sfxSeenProjectiles.insert( proj ).second )
		{
			PlayProjectileLaunchSound( kind, b2Body_GetPosition( proj ) );
		}
	}

	inline void ScheduleProjectileDestroy( b2BodyId id, double when )
	{
		auto it = m_projectilesToDestroyMap.find( id );
		if ( it == m_projectilesToDestroyMap.end() || when < it->second )
			m_projectilesToDestroyMap[id] = when;
	}

	inline void PurgeProjectile( b2BodyId body, bool destroyBody = true )
	{
		if ( !B2_IS_NON_NULL( body ) )
			return;

		EraseBodyFromProjectileSets( body );
		m_projectilesToDestroyMap.erase( body );
		m_projectileSpawnPos.erase( body );

		m_shurikenReboundsLeft.erase( body );
		m_weaponOwner.erase( b2StoreBodyId( body ) );
		ClearPairStateForBody( body );

		m_sfxSeenProjectiles.erase( body );
		if ( !b2Body_IsValid( body ) )
			return;
		if ( destroyBody )
			SafeDestroyBody( body );
	}

	inline void AddPoison( uint64_t victimCharId, int stacks, double now )
	{
		if ( stacks <= 0 )
			stacks = 1;
		auto& p = m_poisonBuildUp[victimCharId]; // p = {lastTick, ticksLeft}
		if ( p.second <= 0 )
			p.first = now;	// démarre la fenêtre si pas actif
		p.second += stacks; // ajoute des ticks (1 dmg par tick)
	}

	inline void AddSlash( uint64_t victimCharId, int ticks, double now )
	{
		if ( ticks <= 0 )
			ticks = 1;
		auto& s = m_slashBuildUp[victimCharId]; // s = { lastTickTime, ticksLeft }
		if ( s.second <= 0 )
		{
			// Première activation : on démarre la cadence. Comme le poison,
			// le 1er tick arrivera après m_slashTickInterval (pas de tick immédiat).
			s.first = now;
		}
		s.second += ticks; // on ajoute des ticks à délivrer (1 dégât / tick)
	}

	void HandleProjectileWallContacts()
	{
		b2ContactEvents ev = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < ev.beginCount; ++i )
		{
			const b2ContactBeginTouchEvent& e = ev.beginEvents[i];
			if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
				continue;

			b2BodyId bodyA = b2Shape_GetBody( e.shapeIdA );
			b2BodyId bodyB = b2Shape_GetBody( e.shapeIdB );
			b2Filter fa = b2Shape_GetFilter( e.shapeIdA );
			b2Filter fb = b2Shape_GetFilter( e.shapeIdB );

			bool isProjA = ( fa.categoryBits & CATEGORY_PROJECTILE ) != 0;
			bool isProjB = ( fb.categoryBits & CATEGORY_PROJECTILE ) != 0;
			bool isWallA = ( fa.categoryBits & CATEGORY_WALL ) != 0;
			bool isWallB = ( fb.categoryBits & CATEGORY_WALL ) != 0;

			if ( !( ( isProjA && isWallB ) || ( isProjB && isWallA ) ) )
				continue;

			b2BodyId proj = isProjA ? bodyA : bodyB;

			if ( m_projectileShuriken.count( proj ) )
			{
				int& r = m_shurikenReboundsLeft[proj];
				if ( --r < 0 )
					m_projectilesToDestroyMap[proj] = ImGui::GetTime();
			}
			else
			{
				m_projectilesToDestroyMap[proj] = ImGui::GetTime();
			}
		}
	}

	void HandleProjectileWeaponContacts()
	{
		const double now = ImGui::GetTime();
		b2ContactEvents ev = b2World_GetContactEvents( m_worldId );

		for ( int i = 0; i < ev.beginCount; ++i )
		{
			const b2ContactBeginTouchEvent& e = ev.beginEvents[i];
			if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
				continue;

			const b2Filter fa = b2Shape_GetFilter( e.shapeIdA );
			const b2Filter fb = b2Shape_GetFilter( e.shapeIdB );

			const bool isProjA = ( fa.categoryBits & CATEGORY_PROJECTILE ) != 0;
			const bool isProjB = ( fb.categoryBits & CATEGORY_PROJECTILE ) != 0;
			const bool isWeapA = ( fa.categoryBits & CATEGORY_WEAPON ) != 0;
			const bool isWeapB = ( fb.categoryBits & CATEGORY_WEAPON ) != 0;
			const bool isTurretA = ( fa.categoryBits & CATEGORY_TURRET ) != 0;
			const bool isTurretB = ( fb.categoryBits & CATEGORY_TURRET ) != 0;

			if ( !( ( isProjA && ( isWeapB || isTurretB ) ) || ( isProjB && ( isWeapA || isTurretA ) ) ) )
				continue;

			b2BodyId proj = isProjA ? b2Shape_GetBody( e.shapeIdA ) : b2Shape_GetBody( e.shapeIdB );
			if ( !B2_IS_NON_NULL( proj ) )
				continue;

			// --- SON D'IMPACT ---
			float speed = 0.f;
			if ( BodyValid( proj ) )
			{
				b2Vec2 v = b2Body_GetLinearVelocity( proj );
				speed = b2Length( v );
			}
			b2Vec2 posA = b2Body_GetPosition( b2Shape_GetBody( e.shapeIdA ) );
			b2Vec2 posB = b2Body_GetPosition( b2Shape_GetBody( e.shapeIdB ) );
			b2Vec2 mid = 0.5f * ( posA + posB );

			// Anti-spam (0.1 s) par projectile
			uint64_t key = b2StoreBodyId( proj );
			double& next = m_nextReboundSoundAllowed[key];
			if ( now >= next )
			{
				next = now + 0.1;
				// au choix : "ping" générique d'impact
				PlayReboundSound( mid, std::max( 0.1f, speed ) );
				// ou un "clank" de rebond :
				// PlayReboundSound(mid, std::max(0.1f, speed));
			}

			// --- logique existante : rebonds/destruction ---
			if ( m_projectileShuriken.count( proj ) )
			{
				int& r = m_shurikenReboundsLeft[proj];
				if ( --r < 0 )
					m_projectilesToDestroyMap[proj] = now;
			}
			else
			{
				const double when = now + 0.5;
				auto it = m_projectilesToDestroyMap.find( proj );
				if ( it == m_projectilesToDestroyMap.end() || when < it->second )
					m_projectilesToDestroyMap[proj] = when;
			}
		}
	}

	void FreezeBodyAndJoint( b2BodyId body, b2JointId joint, double duration )
	{
		if ( !BodyValid( body ) )
			return;
		if ( IsProjectileBodyFast( body ) )
			return; // pas de gel pour les projectiles

		const double newEndTime = ImGui::GetTime() + duration;

		for ( auto& f : m_activeFreezes )
		{
			if ( f.body == body )
			{
				f.endTime = std::max( f.endTime, newEndTime );
				return;
			}
		}

		FreezeData data;
		data.body = body;
		data.joint = joint;

		data.savedLinearVelocity = b2Body_GetLinearVelocity( body );
		data.savedAngularVelocity = b2Body_GetAngularVelocity( body );
		data.endTime = newEndTime;

		data.wasAwake = b2Body_IsAwake( body );
		data.savedSleepThreshold = b2Body_GetSleepThreshold( body );

		// Met immédiatement à 0 et endort (la boucle d’enforcement maintiendra l’état)
		b2Body_SetLinearVelocity( body, { 0.f, 0.f } );
		b2Body_SetAngularVelocity( body, 0.f );
		b2Body_SetAwake( body, false );

		// Moteur du joint fourni (si c’est bien un revolute motor)
		data.hadMotor = false;
		if ( JointValid( joint ) && b2Joint_GetType( joint ) == b2_revoluteJoint )
		{
			data.hadMotor = true;
			data.motorWasEnabled = b2RevoluteJoint_IsMotorEnabled( joint );
			data.savedMotorSpeed = b2RevoluteJoint_GetMotorSpeed( joint );
			data.savedMaxMotorTorque = b2RevoluteJoint_GetMaxMotorTorque( joint );
			b2RevoluteJoint_EnableMotor( joint, false );
		}

		m_activeFreezes.push_back( data );
	}

	void FreezeCharacterAndWeapon( b2BodyId characterBody, double duration )
	{
		if ( !BodyValid( characterBody ) )
			return;
		FreezeBodyAndJoint( characterBody, b2_nullJointId, duration );
		const uint64_t charKey = b2StoreBodyId( characterBody );
		auto itW = m_characterWeapon.find( charKey );
		if ( itW != m_characterWeapon.end() )
		{
			b2BodyId weaponBody = itW->second;
			if ( BodyValid( weaponBody ) )
			{
				const uint64_t wKey = b2StoreBodyId( weaponBody );
				b2JointId j = b2_nullJointId;
				if ( auto itJ = m_weaponToJoint.find( wKey ); itJ != m_weaponToJoint.end() )
					j = itJ->second;

				FreezeBodyAndJoint( weaponBody, j, duration );
			}
		}
	}

	// Clamp util
	static inline b2Vec2 ClampLen( b2Vec2 v, float maxLen )
	{
		const float n = b2Length( v );
		if ( n > maxLen && n > 1e-6f )
			v = ( maxLen / n ) * v;
		return v;
	}
	static inline bool IsProjectileBodyFast( b2BodyId body )
	{
		if ( !BodyValid( body ) )
			return false;
		b2ShapeId sh[1];
		const int n = b2Body_GetShapes( body, sh, 1 );
		if ( n <= 0 )
			return false;
		const b2Filter f = b2Shape_GetFilter( sh[0] );
		return ( f.categoryBits & CATEGORY_PROJECTILE ) != 0;
	}

	void UpdateFreezes()
	{
		const double now = ImGui::GetTime();
		std::vector<FreezeData> still;
		still.reserve( m_activeFreezes.size() );

		for ( FreezeData& f : m_activeFreezes )
		{
			if ( !BodyValid( f.body ) )
				continue;

			if ( now < f.endTime )
			{
				// >>> ENFORCE FREEZE CHAQUE FRAME <<<
				// (évite drift si un contact tente de réveiller le body)
				b2Body_SetLinearVelocity( f.body, { 0.f, 0.f } );
				b2Body_SetAngularVelocity( f.body, 0.f );
				b2Body_SetAwake( f.body, false );

				// Si quelqu’un a réactivé le moteur entre-temps, on le coupe de nouveau
				if ( JointValid( f.joint ) && f.hadMotor && b2Joint_GetType( f.joint ) == b2_revoluteJoint )
				{
					if ( b2RevoluteJoint_IsMotorEnabled( f.joint ) )
						b2RevoluteJoint_EnableMotor( f.joint, false );
				}

				still.push_back( f );
				continue;
			}

			// ====== Fin de gel : reprise propre ======
			// 1) Restituer threshold de sommeil
			b2Body_SetSleepThreshold( f.body, f.savedSleepThreshold );

			// 2) Clamp des vitesses restituées pour éviter un “kick” extrême
			//    (garde la sensation de reprise nette, sans spikes)
			constexpr float kResumeLinClamp = 80.0f; // ajuste à ton échelle
			constexpr float kResumeAngClamp = 40.0f; // rad/s max
			b2Vec2 vLin = ClampLen( f.savedLinearVelocity, kResumeLinClamp );
			float vAng = std::clamp( f.savedAngularVelocity, -kResumeAngClamp, kResumeAngClamp );
			b2Body_SetLinearVelocity( f.body, vLin );
			b2Body_SetAngularVelocity( f.body, vAng );

			// 3) Rétablir le moteur du joint s’il existait
			if ( JointValid( f.joint ) && f.hadMotor && b2Joint_GetType( f.joint ) == b2_revoluteJoint )
			{
				b2RevoluteJoint_SetMaxMotorTorque( f.joint, f.savedMaxMotorTorque );
				b2RevoluteJoint_SetMotorSpeed( f.joint, f.savedMotorSpeed );
				b2RevoluteJoint_EnableMotor( f.joint, f.motorWasEnabled );
			}

			// 4) Réveiller pour repartir immédiatement
			b2Body_SetAwake( f.body, true );
		}

		m_activeFreezes.swap( still );
	}

	void ApplyHitFreeze( b2BodyId attacker, b2BodyId victim )
	{
		FreezeCharacterAndWeapon( attacker, kHitFreezeDuration );
		FreezeCharacterAndWeapon( victim, kHitFreezeDuration );
	}

	void ResolveWeaponMotorInversions( double now )
	{
		b2ContactEvents ev = b2World_GetContactEvents( m_worldId );
		std::unordered_set<PairKeySym, PairKeySymHash, PairKeySymEq> weaponPairsProcessed;
		m_flipLatched.clear();

		for ( int i = 0; i < ev.beginCount; ++i )
		{
			const auto& e = ev.beginEvents[i];
			if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
				continue;
			if ( b2Shape_IsSensor( e.shapeIdA ) || b2Shape_IsSensor( e.shapeIdB ) )
				continue;

			b2BodyId a = b2Shape_GetBody( e.shapeIdA );
			b2BodyId b = b2Shape_GetBody( e.shapeIdB );
			if ( !BodyValid( a ) || !BodyValid( b ) )
				continue;

			if ( IsBodyCurrentlyFrozen( a ) || IsBodyCurrentlyFrozen( b ) )
				continue;
			if ( b2Body_GetType( a ) != b2_dynamicBody || b2Body_GetType( b ) != b2_dynamicBody )
				continue;

			uint64_t idA = b2StoreBodyId( a ), idB = b2StoreBodyId( b );
			auto itA = m_weaponToJoint.find( idA );
			auto itB = m_weaponToJoint.find( idB );
			if ( itA == m_weaponToJoint.end() || itB == m_weaponToJoint.end() )
				continue;

			b2JointId jA = itA->second, jB = itB->second;
			if ( !JointValid( jA ) || !JointValid( jB ) )
				continue;

			auto oA = m_weaponOwner.find( idA );
			if ( oA != m_weaponOwner.end() && BodyValid( oA->second ) && IsBodyCurrentlyFrozen( oA->second ) )
				continue;
			auto oB = m_weaponOwner.find( idB );
			if ( oB != m_weaponOwner.end() && BodyValid( oB->second ) && IsBodyCurrentlyFrozen( oB->second ) )
				continue;

			uint64_t lo = std::min( idA, idB ), hi = std::max( idA, idB );
			PairKeySym sk = MakePairKeySym( idA, idB );
			if ( !weaponPairsProcessed.insert( sk ).second )
				continue;

			if ( !m_flipLatched.insert( idA ).second )
				continue;
			if ( !m_flipLatched.insert( idB ).second )
				continue;

			double& last = m_lastPairSwitchTime[sk];
			if ( now - last < kMotorFlipCooldown )
				continue;

			const float kMinAbs = 0.05f;
			float sA = b2RevoluteJoint_GetMotorSpeed( jA );
			float sB = b2RevoluteJoint_GetMotorSpeed( jB );
			bool did = false;

			if ( std::abs( sA ) >= kMinAbs )
			{
				b2RevoluteJoint_EnableMotor( jA, false );
				b2RevoluteJoint_EnableMotor( jA, true );
				b2RevoluteJoint_SetMotorSpeed( jA, -sA ); // flip sign, keep magnitude
				did = true;
			}
			if ( std::abs( sB ) >= kMinAbs )
			{
				b2RevoluteJoint_EnableMotor( jB, false );
				b2RevoluteJoint_EnableMotor( jB, true );
				b2RevoluteJoint_SetMotorSpeed( jB, -sB );
				did = true;
			}

			if ( did )
			{
				last = now;

				b2Vec2 mid = 0.5f * ( b2Body_GetPosition( a ) + b2Body_GetPosition( b ) );
				PlayMotorInvertSound( mid );

				if ( idA == b2StoreBodyId( m_hammerId ) || idB == b2StoreBodyId( m_hammerId ) )
					m_hammerJustInverted = true;
			}
		}
	}

	void TrackWeaponWeaponSticks( double now )
	{
		b2ContactEvents ev = b2World_GetContactEvents( m_worldId );

		for ( int i = 0; i < ev.beginCount; ++i )
		{
			const b2ContactBeginTouchEvent& e = ev.beginEvents[i];
			if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
				continue;
			b2Filter fa = b2Shape_GetFilter( e.shapeIdA );
			b2Filter fb = b2Shape_GetFilter( e.shapeIdB );

			bool aW = ( fa.categoryBits & CATEGORY_WEAPON ) && !( fa.categoryBits & CATEGORY_PROJECTILE );
			bool bW = ( fb.categoryBits & CATEGORY_WEAPON ) && !( fb.categoryBits & CATEGORY_PROJECTILE );
			if ( !aW || !bW )
				continue;

			uint64_t idA = b2StoreBodyId( b2Shape_GetBody( e.shapeIdA ) );
			uint64_t idB = b2StoreBodyId( b2Shape_GetBody( e.shapeIdB ) );
			PairKeySym key = MakePairKeySym( idA, idB );
			m_weaponPairContactTime[key] = now;
		}

		for ( int i = 0; i < ev.endCount; ++i )
		{
			const b2ContactEndTouchEvent& e = ev.endEvents[i];
			if ( !b2Shape_IsValid( e.shapeIdA ) || !b2Shape_IsValid( e.shapeIdB ) )
				continue;
			b2Filter fa = b2Shape_GetFilter( e.shapeIdA );
			b2Filter fb = b2Shape_GetFilter( e.shapeIdB );

			bool aW = ( fa.categoryBits & CATEGORY_WEAPON ) && !( fa.categoryBits & CATEGORY_PROJECTILE );
			bool bW = ( fb.categoryBits & CATEGORY_WEAPON ) && !( fb.categoryBits & CATEGORY_PROJECTILE );
			if ( !aW || !bW )
				continue;

			uint64_t idA = b2StoreBodyId( b2Shape_GetBody( e.shapeIdA ) );
			uint64_t idB = b2StoreBodyId( b2Shape_GetBody( e.shapeIdB ) );
			PairKeySym key = MakePairKeySym( idA, idB );
			m_weaponPairContactTime.erase( key );
		}
	}

	void HandleCharacterReboundContacts()
	{
		b2ContactEvents contactEvents = b2World_GetContactEvents( m_worldId );

		for ( int i = 0; i < contactEvents.beginCount; ++i )
		{
			const b2ContactBeginTouchEvent& evt = contactEvents.beginEvents[i];
			if ( !b2Shape_IsValid( evt.shapeIdA ) || !b2Shape_IsValid( evt.shapeIdB ) )
				continue;

			b2ShapeId shapeA = evt.shapeIdA;
			b2ShapeId shapeB = evt.shapeIdB;
			b2Filter filterA = b2Shape_GetFilter( shapeA );
			b2Filter filterB = b2Shape_GetFilter( shapeB );

			// Tags utiles
			const bool aIsChar = ( filterA.categoryBits & CATEGORY_CHARACTER ) != 0;
			const bool bIsChar = ( filterB.categoryBits & CATEGORY_CHARACTER ) != 0;
			const bool aIsTurret = ( filterA.categoryBits & CATEGORY_TURRET ) != 0;
			const bool bIsTurret = ( filterB.categoryBits & CATEGORY_TURRET ) != 0;
			const bool aIsWall = ( filterA.categoryBits & CATEGORY_WALL ) != 0;
			const bool bIsWall = ( filterB.categoryBits & CATEGORY_WALL ) != 0;
			const bool aIsSensor = b2Shape_IsSensor( shapeA );
			const bool bIsSensor = b2Shape_IsSensor( shapeB );

			// ─────────────────────────────────────────────────────────────────
			// SON #1 : Character ↔ Wall  (Turret ↔ Wall est silencieux)
			// ─────────────────────────────────────────────────────────────────
			if ( ( aIsChar && bIsWall ) || ( bIsChar && aIsWall ) )
			{
				b2BodyId rebounderBody = aIsChar ? b2Shape_GetBody( shapeA ) : b2Shape_GetBody( shapeB );
				if ( !B2_IS_NON_NULL( rebounderBody ) )
					continue;

				float speed = b2Length( b2Body_GetLinearVelocity( rebounderBody ) );
				if ( speed > 0.1f ) // seuil sonore
				{
					uint64_t key = b2StoreBodyId( rebounderBody );
					double& nextAllowed = m_nextReboundSoundAllowed[key];
					double now = ImGui::GetTime();
					if ( now >= nextAllowed )
					{
						nextAllowed = now + 0.1;
						b2Vec2 posA = b2Body_GetPosition( b2Shape_GetBody( shapeA ) );
						b2Vec2 posB = b2Body_GetPosition( b2Shape_GetBody( shapeB ) );
						PlayReboundSound( 0.5f * ( posA + posB ), speed );
					}
				}
			}

			// ─────────────────────────────────────────────────────────────────
			// SON #2 : Character ↔ Turret (préservé)
			// ─────────────────────────────────────────────────────────────────
			if ( ( ( aIsChar && bIsTurret ) || ( bIsChar && aIsTurret ) ) && !aIsSensor && !bIsSensor )
			{
				b2BodyId charBody = aIsChar ? b2Shape_GetBody( shapeA ) : b2Shape_GetBody( shapeB );
				b2BodyId turretBody = aIsTurret ? b2Shape_GetBody( shapeA ) : b2Shape_GetBody( shapeB );
				if ( !B2_IS_NON_NULL( charBody ) || !B2_IS_NON_NULL( turretBody ) )
					continue;

				float speed = b2Length( b2Body_GetLinearVelocity( charBody ) );
				if ( speed <= 0.01f )
					continue;

				uint64_t key = b2StoreBodyId( charBody );
				double& nextAllowed = m_nextReboundSoundAllowed[key];
				double now = ImGui::GetTime();
				if ( now < nextAllowed )
					continue;
				nextAllowed = now + 0.1;

				b2Vec2 posA = b2Body_GetPosition( charBody );
				b2Vec2 posB = b2Body_GetPosition( turretBody );
				PlayReboundSound( 0.5f * ( posA + posB ), speed );
			}
		}
	}

	void TriggerExplosion( b2BodyId projId )
	{
		if ( !BodyValid( projId ) )
			return;

		b2Vec2 pos = b2Body_GetPosition( projId );

		// Dégâts de base ou overrides par arme/projo
		int damage = m_explosionDamage;
		if ( auto itDmg = m_weaponDamage.find( b2StoreBodyId( projId ) ); itDmg != m_weaponDamage.end() && itDmg->second > 0 )
		{
			damage = itDmg->second;
		}

		// Owner de l'explosion (corps du perso)
		uint64_t ownerCharStored = 0;
		if ( auto itOwner = m_weaponOwner.find( b2StoreBodyId( projId ) ); itOwner != m_weaponOwner.end() )
			ownerCharStored = b2StoreBodyId( itOwner->second );

		// FX anim côté UI
		m_activeExplosions.push_back( { pos, ImGui::GetTime(), m_explosionRadius, damage, ownerCharStored } );
		{
			// Intensité son: combine rayon, magnitude, dégâts (ressenti “plus gros”)
			float sfxPower = 0.6f * m_explosionMagnitude + 0.3f * m_explosionRadius + 0.1f * damage;

			double nowSfx = ImGui::GetTime();
			if ( nowSfx > 0.08 ) // léger cooldown pour éviter les doublons
			{
				PlayExplosionBlast( pos, sfxPower );
			}
		}

		// Physique (impulse)
		b2ExplosionDef def = b2DefaultExplosionDef();
		def.position = pos;
		def.radius = m_explosionRadius;
		def.falloff = 0.2f;
		def.impulsePerLength = m_explosionMagnitude;
		b2World_Explode( m_worldId, &def );

		bool didDamage = false;

		// Dégâts + blink + hitfreeze victime + kill si HP==0
		for ( const std::string& name : kAllCharacters )
		{
			b2BodyId charId = GetCharacterIdByName( name );
			if ( !B2_IS_NON_NULL( charId ) )
				continue;
			if ( b2StoreBodyId( charId ) == ownerCharStored )
				continue; // pas d'auto-dégât

			if ( b2Distance( b2Body_GetPosition( charId ), pos ) <= m_explosionRadius + kCharacterRadius )
			{
				auto itHP = m_characterHP.find( b2StoreBodyId( charId ) );
				if ( itHP != m_characterHP.end() && itHP->second > 0 )
				{
					const int hpBefore = itHP->second;
					itHP->second = std::max( 0, itHP->second - damage );

					if ( itHP->second < hpBefore )
					{
						didDamage = true;

						// Feedback victime
						m_lastHitBlinkTime[b2StoreBodyId( charId )] = ImGui::GetTime();
						if ( !IsBodyCurrentlyFrozen( charId ) )
							FreezeCharacterAndWeapon( charId, m_projectileHitFreeze );

						// Mort ?
						if ( itHP->second == 0 )
							RequestKillCharacter( charId );
					}
				}
			}
		}

		// Si au moins une cible a pris des dégâts : progression + son + hitfreeze de l'owner
		if ( didDamage )
		{
			m_explosionRadius = std::min( m_explosionRadius + 0.5f, 10.0f );
			m_explosionMagnitude = std::min( m_explosionMagnitude + 0.5f, 10.0f );
			m_explosionDamage = std::min( m_explosionDamage + 1, 10 );

			// Attaquant (owner) : hitfreeze comme un échange de coups
			b2BodyId ownerBody = GetCharacterBodyByStoredId( ownerCharStored );
			if ( BodyValid( ownerBody ) && !IsBodyCurrentlyFrozen( ownerBody ) )
				FreezeCharacterAndWeapon( ownerBody, m_projectileHitFreeze );
		}
	}

	void UpdateBowPassives()
	{
	}
	void UpdateCrossbowPassives()
	{
		if ( !B2_IS_NON_NULL( m_crossbowId ) )
			return;
		uint64_t id = b2StoreBodyId( m_crossbowId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateSwordPassives()
	{
		if ( !B2_IS_NON_NULL( m_swordId ) )
			return;
		uint64_t id = b2StoreBodyId( m_swordId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateAxePassives()
	{
		if ( !B2_IS_NON_NULL( m_axeId ) )
			return;
		uint64_t id = b2StoreBodyId( m_axeId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateDaggerPassives()
	{
	}
	void UpdateKatanaPassives()
	{
		// +1 stack de slash ajouté par coup suivant
		++m_katanaSlashStacks;
	}

	void UpdateScythePassives()
	{
		// +1 stack de poison ajouté par coup suivant
		++m_scythePoisonStacks;
	}

	void UpdateTridentPassives()
	{
		if ( !B2_IS_NON_NULL( m_tridentId ) )
			return;
		uint64_t id = b2StoreBodyId( m_tridentId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdatehammerPassives()
	{
	}
	void UpdatePoisonBlowgunPassives()
	{
	}
	void UpdateClubPassives()
	{
		if ( !B2_IS_NON_NULL( m_clubId ) )
			return;
		uint64_t id = b2StoreBodyId( m_clubId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}

	void UpdateSpearPassives()
	{
		if ( !B2_IS_NON_NULL( m_spearId ) )
			return;

		const uint64_t id = b2StoreBodyId( m_spearId );

		// Un upgrade par hit autorisé (le cooldown global fait foi)
		if ( m_spearLevel < kMaxSpearLevel )
		{
			UpgradeSpearSprite(); // -> fixe m_weaponDamage à (1 + m_spearLevel)
		}
		else
		{
			// Au cap, on s'assure que les dégâts restent cohérents
			m_weaponDamage[id] = std::max( 1, 1 + m_spearLevel );
		}
	}

	void UpdateKunaiPassives()
	{
		if ( !B2_IS_NON_NULL( m_kunaiId ) )
			return;
		uint64_t id = b2StoreBodyId( m_kunaiId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateBoomerangPassives( uint64_t weaponKey )
	{
		auto it = m_boomerangs.find( weaponKey );
		if ( it == m_boomerangs.end() )
			return;

		BoomerangState& s = it->second;

		// mêmes règles que ta version “globale”, mais par boomerang
		s.reachA = std::min( s.reachA + 0.25f, 10.0f );
		s.reachB = std::min( s.reachB + 0.25f, 10.0f );

		++s.hitCount;
		if ( s.hitCount % 3 == 0 )
		{
			auto itD = m_weaponDamage.find( weaponKey );
			if ( itD != m_weaponDamage.end() && itD->second > 0 && itD->second < 5 )
				++( itD->second );
		}
	}

	void UpdateShurikenPassives()
	{
		++m_shurikenBonusRebounds;
	}
	void UpdateBigSwordPassives()
	{
		if ( !B2_IS_NON_NULL( m_bigSwordId ) )
			return;
		uint64_t id = b2StoreBodyId( m_bigSwordId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateElectricStaffPassives()
	{
		if ( !B2_IS_NON_NULL( m_electricStaffId ) )
			return;
		uint64_t id = b2StoreBodyId( m_electricStaffId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateExplosionStaffPassives()
	{
	}
	void UpdateFrostStaffPassives()
	{
		if ( !B2_IS_NON_NULL( m_frostStaffId ) )
			return;
		uint64_t id = b2StoreBodyId( m_frostStaffId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateMahoragaPassives()
	{
		if ( !B2_IS_NON_NULL( m_mahoragaId ) )
			return;
		uint64_t id = b2StoreBodyId( m_mahoragaId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateShieldPassives()
	{
		if ( !B2_IS_NON_NULL( m_shieldId ) )
			return;
		uint64_t id = b2StoreBodyId( m_shieldId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateVampireKnifePassives()
	{
		if ( !B2_IS_NON_NULL( m_vampireKnifeId ) )
			return;
		uint64_t id = b2StoreBodyId( m_vampireKnifeId );
		auto it = m_weaponDamage.find( id );
		if ( it != m_weaponDamage.end() && it->second > 0 )
			++( it->second );
	}
	void UpdateWrenchPassives()
	{
	}

	void UpdateUnarmedSpeedRampAndDamage()
	{
		if ( !BodyValid( m_characterIdUnarmed ) )
			return;

		double now = ImGui::GetTime();
		if ( m_unarmedLastT == 0.0 )
			m_unarmedLastT = now;
		double dt = now - m_unarmedLastT;
		m_unarmedLastT = now;

		b2Vec2 v = b2Body_GetLinearVelocity( m_characterIdUnarmed );
		float s = b2Length( v );
		int dmg = std::clamp( int( s * 0.2f ), 1, 12 );
		m_weaponDamage[b2StoreBodyId( m_characterIdUnarmed )] = dmg;
	}
	void UpdatehammerDamageProgression( double now )
	{
		if ( !B2_IS_NON_NULL( m_hammerId ) )
			return;

		uint64_t id = b2StoreBodyId( m_hammerId );
		auto it = m_weaponDamage.find( id );
		if ( it == m_weaponDamage.end() )
			return;

		if ( m_hammerJustInverted )
		{
			it->second = 1;
			m_hammerLastIncTime = now;
		}
		else if ( now - m_hammerLastIncTime > m_hammerIncInterval )
		{
			++( it->second );
			m_hammerLastIncTime = now;
		}
	}

	void UpdateGlaivePassives()
	{
	}
	void UpdateSickleRightPassives()
	{
	}
	void UpdateFlaskPassives()
	{
	}

	void UpdateLancePassives()
	{ /* rien de spécial pour l’instant */
	}

	void UpdateWeaponsPassives( uint64_t weaponId )
	{
		if ( weaponId == b2StoreBodyId( m_bowId ) )
			UpdateBowPassives();
		else if ( weaponId == b2StoreBodyId( m_crossbowId ) )
			UpdateCrossbowPassives();
		else if ( weaponId == b2StoreBodyId( m_swordId ) )
			UpdateSwordPassives();
		else if ( weaponId == b2StoreBodyId( m_axeId ) )
			UpdateAxePassives();
		else if ( weaponId == b2StoreBodyId( m_daggerId ) )
			UpdateDaggerPassives();
		else if ( weaponId == b2StoreBodyId( m_katanaId ) )
			UpdateKatanaPassives();
		else if ( weaponId == b2StoreBodyId( m_tridentId ) )
			UpdateTridentPassives();
		else if ( weaponId == b2StoreBodyId( m_hammerId ) )
			UpdatehammerPassives();
		else if ( weaponId == b2StoreBodyId( m_poisonBlowgunId ) )
			UpdatePoisonBlowgunPassives();
		else if ( weaponId == b2StoreBodyId( m_clubId ) )
			UpdateClubPassives();
		else if ( weaponId == b2StoreBodyId( m_scytheId ) )
			UpdateScythePassives();
		else if ( weaponId == b2StoreBodyId( m_spearId ) )
			UpdateSpearPassives();
		else if ( weaponId == b2StoreBodyId( m_kunaiId ) )
			UpdateKunaiPassives();
		else if ( m_boomerangs.count( weaponId ) )
			UpdateBoomerangPassives( weaponId );
		else if ( weaponId == b2StoreBodyId( m_shurikenId ) )
			UpdateShurikenPassives();
		else if ( weaponId == b2StoreBodyId( m_bigSwordId ) )
			UpdateBigSwordPassives();
		else if ( weaponId == b2StoreBodyId( m_electricStaffId ) )
			UpdateElectricStaffPassives();
		else if ( weaponId == b2StoreBodyId( m_explosionStaffId ) )
			UpdateExplosionStaffPassives();
		else if ( weaponId == b2StoreBodyId( m_frostStaffId ) )
			UpdateFrostStaffPassives();
		else if ( weaponId == b2StoreBodyId( m_mahoragaId ) )
			UpdateMahoragaPassives();
		else if ( weaponId == b2StoreBodyId( m_shieldId ) )
			UpdateShieldPassives();
		else if ( weaponId == b2StoreBodyId( m_vampireKnifeId ) )
			UpdateVampireKnifePassives();
		else if ( weaponId == b2StoreBodyId( m_wrenchId ) )
			UpdateWrenchPassives();
		// NEW
		else if ( weaponId == b2StoreBodyId( m_glaiveId ) )
			UpdateGlaivePassives();
		else if ( weaponId == b2StoreBodyId( m_sickleRId ) )
			UpdateSickleRightPassives();
		else if ( weaponId == b2StoreBodyId( m_flaskId ) )
			UpdateFlaskPassives();
		else if ( weaponId == b2StoreBodyId( m_lanceId ) )
			UpdateLancePassives();
	}

	void UpgradeSpearSprite()
	{
		if ( !B2_IS_NON_NULL( m_spearId ) )
			return;
		if ( m_spearLevel >= kMaxSpearLevel )
			return;

		++m_spearLevel;				   // le pivot dépend de ce niveau
		m_spearPixelSize = kPixelSize; // taille de pixel (constante)

		char spriteName[32];
		snprintf( spriteName, sizeof( spriteName ), "SPEAR_LVL%d", m_spearLevel );
		const PixelArtColor* art = PixelArtColor_GetByName( spriteName );
		if ( !art )
			return;

		BuildSpearFromSprite( art, m_spearPixelSize );
		m_weaponDamage[b2StoreBodyId( m_spearId )] = 1 + m_spearLevel;
	}

	// ========= Accès / helpers de lookup =========

	b2BodyId GetCharacterIdByName( const std::string& name ) const
	{
		static const std::unordered_map<std::string, b2BodyId WeaponsBallsVS::*> charIdMap = {
			{ "Bow", &WeaponsBallsVS::m_characterIdBow },
			{ "Crossbow", &WeaponsBallsVS::m_characterIdCrossbowBow },
			{ "Sword", &WeaponsBallsVS::m_characterIdSword },
			{ "Axe", &WeaponsBallsVS::m_characterIdAxe },
			{ "Dagger", &WeaponsBallsVS::m_characterIdDagger },
			{ "Katana", &WeaponsBallsVS::m_characterIdKatana },
			{ "Trident", &WeaponsBallsVS::m_characterIdTrident },
			{ "Hammer", &WeaponsBallsVS::m_characterIdhammer },
			{ "Poison_Blowgun", &WeaponsBallsVS::m_characterIdPoisonBlowgun },
			{ "Club", &WeaponsBallsVS::m_characterIdClub },
			{ "Scythe", &WeaponsBallsVS::m_characterIdScythe },
			{ "Spear", &WeaponsBallsVS::m_characterIdSpear },
			{ "Kunai", &WeaponsBallsVS::m_characterIdKunai },
			{ "Boomerang", &WeaponsBallsVS::m_characterIdBoomerang },
			{ "Shuriken", &WeaponsBallsVS::m_characterIdShuriken },
			{ "Big_Sword", &WeaponsBallsVS::m_characterIdBigSword },
			{ "Electric_Staff", &WeaponsBallsVS::m_characterIdElectricStaff },
			{ "Explosion_Staff", &WeaponsBallsVS::m_characterIdExplosionStaff },
			{ "Frost_Staff", &WeaponsBallsVS::m_characterIdFrostStaff },
			{ "Mahoraga", &WeaponsBallsVS::m_characterIdMahoraga },
			{ "VampireKnife", &WeaponsBallsVS::m_characterIdVampireKnife },
			{ "Shield", &WeaponsBallsVS::m_characterIdShield },
			{ "Wrench", &WeaponsBallsVS::m_characterIdWrench },
			{ "Unarmed", &WeaponsBallsVS::m_characterIdUnarmed },
			{ "Glaive", &WeaponsBallsVS::m_characterIdGlaive },
			{ "Sickle_R", &WeaponsBallsVS::m_characterIdSickleR },
			{ "Flask", &WeaponsBallsVS::m_characterIdFlask },
			{ "Lance", &WeaponsBallsVS::m_characterIdLance },

		};

		auto it = charIdMap.find( name );
		if ( it != charIdMap.end() )
		{
			return this->*( it->second );
		}
		return b2_nullBodyId;
	}

	b2BodyId GetCharacterBodyByStoredId( uint64_t key ) const
	{
		for ( const std::string& name : kAllCharacters )
		{
			b2BodyId id = GetCharacterIdByName( name );
			if ( BodyValid( id ) && b2StoreBodyId( id ) == key )
				return id;
		}
		return b2_nullBodyId;
	}

	b2BodyId GetOwnerCharacterOfWeapon( b2BodyId weapon ) const
	{
		if ( !B2_IS_NON_NULL( weapon ) )
			return b2_nullBodyId;

		auto itProjOwner = m_weaponOwner.find( b2StoreBodyId( weapon ) );
		if ( itProjOwner != m_weaponOwner.end() )
			return itProjOwner->second;

		for ( const auto& kv : m_characterWeapon )
		{
			if ( B2_IS_NON_NULL( kv.second ) && b2StoreBodyId( kv.second ) == b2StoreBodyId( weapon ) )
				return GetCharacterBodyByStoredId( kv.first );
		}
		return b2_nullBodyId;
	}

	// ========= Spawns & tirs =========

	std::vector<b2Vec2> GenerateSpawnPositions( int count, float radius, b2Vec2 center, std::mt19937& rng )
	{
		std::vector<b2Vec2> positions;
		if ( count == 1 )
		{
			positions.push_back( center );
			return positions;
		}
		float angleStep = 2 * b2_pi / float( count );
		std::uniform_real_distribution<float> angleJitter( -0.15f, 0.15f );
		std::uniform_real_distribution<float> distJitter( -0.2f, 0.2f );

		for ( int i = 0; i < count; ++i )
		{
			float angle = i * angleStep + angleJitter( rng );
			float r = radius + distJitter( rng );
			b2Vec2 pos = { center.x + r * std::cos( angle ), center.y + r * std::sin( angle ) };
			positions.push_back( pos );
		}
		return positions;
	}

	std::vector<b2Vec2> GenerateSpawnPositionsNoJitter( int count, float radius, b2Vec2 center )
	{
		std::vector<b2Vec2> positions;
		if ( count <= 1 )
		{
			positions.push_back( center );
			return positions;
		}

		const float step = 2.0f * b2_pi / float( count );
		for ( int i = 0; i < count; ++i )
		{
			const float a = i * step;
			positions.push_back( { center.x + radius * std::cos( a ), center.y + radius * std::sin( a ) } );
		}
		return positions;
	}

	void LaunchCharacterRandom( b2BodyId body, std::mt19937& rng, float baseMin, float baseMax )
	{
		if ( !BodyValid( body ) )
			return;

		std::uniform_real_distribution<float> ang( 0.0f, 2.0f * b2_pi );
		std::uniform_real_distribution<float> spd( baseMin, baseMax );
		std::uniform_real_distribution<float> spin( -2.0f, 2.0f ); // petit effet de rotation

		const float a = ang( rng );
		const float s = spd( rng );
		const b2Vec2 v = { s * std::cos( a ), s * std::sin( a ) };

		b2Body_SetLinearVelocity( body, v );
		b2Body_SetAngularVelocity( body, spin( rng ) );
	}

	void RandomLaunchAllCharacters( std::mt19937& rng )
	{
		for ( const std::string& name : kAllCharacters )
		{
			b2BodyId id = GetCharacterIdByName( name );
			if ( !BodyValid( id ) )
				continue;
			const float minV = ( id == m_characterIdUnarmed ) ? m_unarmedMinVel : m_minSpeedAll;
			LaunchCharacterRandom( id, rng, minV, minV * 1.4f );
		}
	}

	// ========= Visuel / FX =========

	void ApplyCharacterVisual( b2BodyId charId, float blinkAmount, bool poisoned, float poisonPulse, bool slashed,
							   float slashPulse )
	{
		if ( !BodyValid( charId ) )
			return;

		const uint32_t kWhite = 0xFFFFFF;
		const uint32_t kPoison = 0xAA50FF;	   // violet poison
		const uint32_t kKatanaCyan = 0x00C8C8; // cyan katana

		b2ShapeId shapes[128];
		int nShapes = b2Body_GetShapes( charId, shapes, 128 );
		const uint64_t myCharKey = b2StoreBodyId( charId );

		for ( int i = 0; i < nShapes; ++i )
		{
			b2ShapeId sh = shapes[i];
			b2Filter f = b2Shape_GetFilter( sh );
			uint64_t skey = b2StoreShapeId( sh );

			// Avant : exigeait "belongs" → ça excluait un sensor non mappé.
			const bool isSkinOrChar = ( f.categoryBits & ( CATEGORY_SKIN | CATEGORY_CHARACTER ) ) != 0;
			const bool isOwnedSensor =
				( ( f.categoryBits & ( CATEGORY_HITBOX | CATEGORY_WEAPON ) ) != 0 ) && b2Shape_IsSensor( sh );
			if ( !( isSkinOrChar || isOwnedSensor ) )
				continue;

			// Lazy bind : si c’est un sensor sur CE body, on l’associe au perso ici.
			if ( isOwnedSensor )
				m_shapeToCharacter[skey] = myCharKey;

			// Lazy base color : si on n’a pas de couleur de base mémorisée, on prend celle du mat actuel.
			if ( m_shapeBaseColor.find( skey ) == m_shapeBaseColor.end() )
			{
				b2SurfaceMaterial mat0 = b2Shape_GetSurfaceMaterial( sh );
				m_shapeBaseColor[skey] = mat0.customColor;
			}

			uint32_t base = m_shapeBaseColor[skey];

			// --------------- Nouveau layering couleur ---------------
			// 1) on applique d’abord les teintes "états" (slash / poison)
			uint32_t tinted = base;
			if ( slashed )
				tinted = LerpColor( tinted, kKatanaCyan, std::clamp( slashPulse, 0.f, 1.f ) );
			if ( poisoned )
				tinted = LerpColor( tinted, kPoison, std::clamp( poisonPulse, 0.f, 1.f ) );

			// 2) puis le flash blanc par-dessus (au lieu d’un else-if exclusif)
			uint32_t finalCol = ( blinkAmount > 0.f ) ? LerpColor( tinted, kWhite, std::clamp( blinkAmount, 0.f, 1.f ) ) : tinted;
			// --------------------------------------------------------

			b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( sh );
			mat.customColor = finalCol;
			b2Shape_SetSurfaceMaterial( sh, &mat ); // ✅ passer l’adresse
		}
	}

	void SpawnSlashLines( b2BodyId victimBody, int count )
	{
		if ( !BodyValid( victimBody ) || count <= 0 )
			return;

		const b2Vec2 pos = b2Body_GetPosition( victimBody );
		const double now = ImGui::GetTime();
		const uint64_t vid = b2StoreBodyId( victimBody );

		// graine: mélange temps + id perso + nb de lignes + step counter
		auto mix64 = []( uint64_t x ) {
			x ^= x >> 33;
			x *= 0xff51afd7ed558ccdULL;
			x ^= x >> 33;
			x *= 0xc4ceb9fe1a85ec53ULL;
			x ^= x >> 33;
			return x;
		};
		uint64_t seed64 = (uint64_t)( now * 1e6 ) ^ ( vid * 0x9E3779B97F4A7C15ULL ) ^ (uint64_t)count * 0xBF58476D1CE4E5B9ULL ^
						  (uint64_t)m_stepCounter * 0x94D049BB133111EBULL;
		std::mt19937 rng( (uint32_t)( mix64( seed64 ) ^ ( mix64( seed64 ) >> 32 ) ) );

		// base aléatoire + progression "golden angle"
		std::uniform_real_distribution<float> baseAng( 0.f, 2.f * b2_pi );
		float base = baseAng( rng );
		constexpr float kGolden = 2.39996322972865332f; // ~137.5°

		// variations
		std::uniform_real_distribution<float> jitterAng( -0.6f, +0.6f ); // un peu plus large
		std::uniform_real_distribution<float> jitterLen( 0.85f, 1.25f ); // autour de 4R
		std::uniform_real_distribution<float> jitterOff( -0.8f, +0.8f ); // décalage latéral

		const float unitLen = kCharacterRadius * 4.0f; // moyenne = 4R (comme avant)

		for ( int i = 0; i < count; ++i )
		{
			float ang = base + i * kGolden + jitterAng( rng );
			b2Vec2 dir = { std::cos( ang ), std::sin( ang ) };
			b2Vec2 nrm = { -dir.y, dir.x };

			float len = unitLen * jitterLen( rng );
			b2Vec2 center = pos + nrm * ( kCharacterRadius * 0.35f * jitterOff( rng ) );

			b2Vec2 A = center - dir * ( len * 0.5f );
			b2Vec2 B = center + dir * ( len * 0.5f );

			m_activeSlashLines.push_back( { A, B, now, m_slashFxDuration } );
		}
	}

	// ========= Gestion de vies / morts =========

	inline void RequestKillCharacter( b2BodyId body )
	{
		if ( B2_IS_NON_NULL( body ) )
			m_charactersToKill.push_back( body );
	}

	void KillCharacterNow( b2BodyId charBody )
	{
		if ( !B2_IS_NON_NULL( charBody ) )
			return;

		// 🔵 VFX mort – déclenche AVANT toute purge (on a encore la couleur & la position)
		if ( BodyValid( charBody ) )
			SpawnDeathPoof( charBody );

		const uint64_t charKey = b2StoreBodyId( charBody );

		m_activeFreezes.erase( std::remove_if( m_activeFreezes.begin(), m_activeFreezes.end(),
											   [charBody]( const FreezeData& f ) { return f.body == charBody; } ),
							   m_activeFreezes.end() );
		m_lastHitBlinkTime.erase( charKey );
		m_electricStaffFreezeDuration.erase( charKey );
		m_poisonBuildUp.erase( charKey );
		m_vampireKnifeHealCount.erase( charKey );
		m_vampireKnifeTotalHealed.erase( charKey );

		if ( BodyValid( charBody ) )
		{
			b2ShapeId shapes[128];
			int n = b2Body_GetShapes( charBody, shapes, 128 );
			for ( int i = 0; i < n; ++i )
			{
				const uint64_t skey = b2StoreShapeId( shapes[i] );
				m_shapeToCharacter.erase( skey );
				m_shapeBaseColor.erase( skey );
			}
		}
		m_characterSkinShape.erase( charKey );

		// --- VFX: retirer les leech rays qui suivaient CE perso (en tant qu'owner) ---
		if ( B2_IS_NON_NULL( charBody ) )
		{
			m_leechRays.erase( std::remove_if( m_leechRays.begin(), m_leechRays.end(),
											   [&]( const LeechRay& r ) { return r.toBody == charBody; } ),
							   m_leechRays.end() );
		}

		if ( auto itW = m_characterWeapon.find( charKey ); itW != m_characterWeapon.end() )
		{
			b2BodyId weaponBody = itW->second;
			const uint64_t wKey = b2StoreBodyId( weaponBody );

			m_activeFreezes.erase( std::remove_if( m_activeFreezes.begin(), m_activeFreezes.end(),
												   [weaponBody]( const FreezeData& f ) { return f.body == weaponBody; } ),
								   m_activeFreezes.end() );

			

			if ( BodyValid( weaponBody ) )
			{
				b2ShapeId wShapes[1024];
				int wn = b2Body_GetShapes( weaponBody, wShapes, 1024 );
				for ( int i = 0; i < wn; ++i )
				{
					const uint64_t wskey = b2StoreShapeId( wShapes[i] );
					m_shapeBaseColor.erase( wskey );
					m_shapeToCharacter.erase( wskey );
				}
			}

			m_weaponDamage.erase( wKey );
			m_weaponOwner.erase( wKey );
			ClearWeaponSlotIfMatches( weaponBody );
			SafeDestroyBody( weaponBody );
			m_characterWeapon.erase( itW );
		}

		{
			std::vector<b2BodyId> turrets;
			for ( const auto& kv : m_turretOwner )
				if ( B2_IS_NON_NULL( kv.first ) && B2_IS_NON_NULL( kv.second ) && b2StoreBodyId( kv.second ) == charKey )
					turrets.push_back( kv.first );

			for ( b2BodyId t : turrets )
			{
				m_activeFreezes.erase( std::remove_if( m_activeFreezes.begin(), m_activeFreezes.end(),
													   [t]( const FreezeData& f ) { return f.body == t; } ),
									   m_activeFreezes.end() );
				DestroyTurret( t );
			}
		}

		{
			std::vector<b2BodyId> owned;
			ForEachProjectileBody( [&]( b2BodyId proj ) {
				const uint64_t pKey = b2StoreBodyId( proj );
				auto itO = m_weaponOwner.find( pKey );
				if ( itO != m_weaponOwner.end() && B2_IS_NON_NULL( itO->second ) && b2StoreBodyId( itO->second ) == charKey )
					owned.push_back( proj );
			} );
			const double now = ImGui::GetTime();
			for ( b2BodyId p : owned )
			{
				PurgeProjectile( p, /*destroyBody=*/false );
				ScheduleProjectileDestroy( p, now );
			}
		}

		{
			std::vector<PairKey> toErase;
			for ( const auto& kv : m_damageCooldown )
			{
				const PairKey& k = kv.first;
				if ( k.victim == charKey || k.attacker == charKey )
					toErase.push_back( k );
			}
			for ( const PairKey& k : toErase )
				m_damageCooldown.erase( k );
		}

		{
			std::vector<PairKey> toErase2;
			for ( const auto& kv : m_pairOverlap )
			{
				const PairKey& k = kv.first;
				if ( k.victim == charKey || k.attacker == charKey )
					toErase2.push_back( k );
			}
			for ( const PairKey& k : toErase2 )
				m_pairOverlap.erase( k );
		}

		auto clearIfMatch = [&]( b2BodyId& slot ) {
			if ( slot == charBody )
				slot = b2_nullBodyId;
		};
		clearIfMatch( m_characterIdBow );
		clearIfMatch( m_characterIdCrossbowBow );
		clearIfMatch( m_characterIdSword );
		clearIfMatch( m_characterIdAxe );
		clearIfMatch( m_characterIdDagger );
		clearIfMatch( m_characterIdKatana );
		clearIfMatch( m_characterIdTrident );
		clearIfMatch( m_characterIdhammer );
		clearIfMatch( m_characterIdPoisonBlowgun );
		clearIfMatch( m_characterIdClub );
		clearIfMatch( m_characterIdScythe );
		clearIfMatch( m_characterIdSpear );
		clearIfMatch( m_characterIdKunai );
		clearIfMatch( m_characterIdBoomerang );
		clearIfMatch( m_characterIdShuriken );
		clearIfMatch( m_characterIdBigSword );
		clearIfMatch( m_characterIdElectricStaff );
		clearIfMatch( m_characterIdExplosionStaff );
		clearIfMatch( m_characterIdFrostStaff );
		clearIfMatch( m_characterIdMahoraga );
		clearIfMatch( m_characterIdShield );
		clearIfMatch( m_characterIdVampireKnife );
		clearIfMatch( m_characterIdWrench );
		// NEW
		clearIfMatch( m_characterIdGlaive );
		clearIfMatch( m_characterIdSickleR );
		clearIfMatch( m_characterIdFlask );

		m_characterHP.erase( charKey );
		SafeDestroyBody( charBody );
	}

	void ClearWeaponSlotIfMatches( b2BodyId weapon )
	{
		auto clr = [&]( b2BodyId& slot ) {
			if ( slot == weapon )
				slot = b2_nullBodyId;
		};
		clr( m_bowId );
		clr( m_crossbowId );
		clr( m_swordId );
		clr( m_axeId );
		clr( m_daggerId );
		clr( m_katanaId );
		clr( m_tridentId );
		clr( m_hammerId );
		clr( m_poisonBlowgunId );
		clr( m_clubId );
		clr( m_scytheId );
		clr( m_spearId );
		clr( m_kunaiId );
		clr( m_boomerangId );
		clr( m_shurikenId );
		clr( m_bigSwordId );
		clr( m_electricStaffId );
		clr( m_explosionStaffId );
		clr( m_frostStaffId );
		clr( m_mahoragaId );
		clr( m_vampireKnifeId );
		clr( m_shieldId );
		clr( m_wrenchId );
		clr( m_glaiveId );
		clr( m_sickleRId );
		clr( m_flaskId );
		clr( m_lanceId );
	}

	// ========= Outils bas niveau =========

	static inline bool BodyValid( b2BodyId id )
	{
		return B2_IS_NON_NULL( id ) && b2Body_IsValid( id );
	}
	static inline bool JointValid( b2JointId id )
	{
		return B2_IS_NON_NULL( id ) && b2Joint_IsValid( id );
	}
	static inline int16_t MakeTurretGroup( b2BodyId turretId )
	{
		return -(int16_t)( ( (int32_t)b2StoreBodyId( turretId ) & 0x7FFF ) + 1 );
	}

	void ClearPairStateForBody( b2BodyId body )
	{
		if ( !B2_IS_NON_NULL( body ) )
			return;
		const uint64_t key64 = b2StoreBodyId( body );

		// m_pairOverlap & m_damageCooldown (clés orientées)
		{
			std::vector<PairKey> toErase;
			toErase.reserve( m_pairOverlap.size() );
			for ( const auto& kv : m_pairOverlap )
			{
				const PairKey& k = kv.first;
				if ( k.attacker == key64 || k.victim == key64 ) // victim = characterId ; attacker = bodyId
					toErase.push_back( k );
			}
			for ( const PairKey& k : toErase )
				m_pairOverlap.erase( k );
		}
		{
			std::vector<PairKey> toErase;
			toErase.reserve( m_damageCooldown.size() );
			for ( const auto& kv : m_damageCooldown )
			{
				const PairKey& k = kv.first;
				if ( k.attacker == key64 || k.victim == key64 )
					toErase.push_back( k );
			}
			for ( const PairKey& k : toErase )
				m_damageCooldown.erase( k );
		}

		// Paire symétrique (armes↔armes)
		{
			std::vector<PairKeySym> toErase;
			toErase.reserve( m_lastPairSwitchTime.size() );
			for ( const auto& kv : m_lastPairSwitchTime )
			{
				const PairKeySym& k = kv.first;
				if ( k.lo == key64 || k.hi == key64 )
					toErase.push_back( k );
			}
			for ( const auto& k : toErase )
				m_lastPairSwitchTime.erase( k );
		}
		{
			std::vector<PairKeySym> toErase;
			toErase.reserve( m_weaponPairContactTime.size() );
			for ( const auto& kv : m_weaponPairContactTime )
			{
				const PairKeySym& k = kv.first;
				if ( k.lo == key64 || k.hi == key64 )
					toErase.push_back( k );
			}
			for ( const auto& k : toErase )
				m_weaponPairContactTime.erase( k );
		}
	}

	static inline uint32_t LerpColor( uint32_t a, uint32_t b, float t )
	{
		t = std::clamp( t, 0.0f, 1.0f );
		uint8_t ar = ( a >> 16 ) & 0xFF, ag = ( a >> 8 ) & 0xFF, ab = a & 0xFF;
		uint8_t br = ( b >> 16 ) & 0xFF, bg = ( b >> 8 ) & 0xFF, bb = b & 0xFF;

		auto lerp8 = [&]( uint8_t x, uint8_t y ) { return (uint8_t)std::round( x + ( y - x ) * t ); };

		return ( uint32_t( lerp8( ar, br ) ) << 16 ) | ( uint32_t( lerp8( ag, bg ) ) << 8 ) | uint32_t( lerp8( ab, bb ) );
	}
	void EnsureMinVelocity( b2BodyId body, float minVel )
	{
		if ( !BodyValid( body ) )
			return;

		const float thresh = std::max( minVel, b2Body_GetSleepThreshold( body ) );
		b2Vec2 v = b2Body_GetLinearVelocity( body );
		float n = b2Length( v );

		if ( n < 1e-3f )
		{
			b2Body_SetAwake( body, true );
			b2Body_SetLinearVelocity( body, { thresh, 0.f } );
			return;
		}

		if ( n < thresh )
		{
			b2Vec2 dir = ( 1.0f / n ) * v;
			b2Body_SetAwake( body, true );
			b2Body_SetLinearVelocity( body, thresh * dir );
		}
	}
	bool IsBodyCurrentlyFrozen( b2BodyId body ) const
	{
		for ( const FreezeData& f : m_activeFreezes )
			if ( f.body == body )
				return true;
		return false;
	}
	bool IsSpawnTooClose( const b2Vec2& spawnPos, b2BodyId shooterId, float minDist = 1.0f )
	{
		b2BodyId ids[] = {
			m_characterIdBow,			m_characterIdCrossbowBow,	 m_characterIdSword,	  m_characterIdAxe,
			m_characterIdDagger,		m_characterIdKatana,		 m_characterIdTrident,	  m_characterIdhammer,
			m_characterIdPoisonBlowgun, m_characterIdClub,			 m_characterIdScythe,	  m_characterIdSpear,
			m_characterIdKunai,			m_characterIdBoomerang,		 m_characterIdShuriken,	  m_characterIdBigSword,
			m_characterIdElectricStaff, m_characterIdExplosionStaff, m_characterIdFrostStaff, m_characterIdMahoraga,
			m_characterIdVampireKnife,	m_characterIdShield,		 m_characterIdWrench,	  m_characterIdUnarmed,
			m_characterIdGlaive,		m_characterIdSickleR,		 m_characterIdFlask,	  m_characterIdLance };

		for ( b2BodyId charId : ids )
		{
			if ( !BodyValid( charId ) || charId == shooterId )
				continue;
			float dist = b2Distance( b2Body_GetPosition( charId ), spawnPos );
			if ( dist < minDist )
				return true;
		}
		return false;
	}
	int FindMeleeBankFor( b2BodyId weapon ) const
	{
		if ( !B2_IS_NON_NULL( weapon ) )
			return m_banks.w_unarmed;

		const struct Pair
		{
			b2BodyId id;
			int bank;
		} pairs[] = {
			{ m_bowId, m_banks.w_bow },
			{ m_crossbowId, m_banks.w_crossbow },
			{ m_swordId, m_banks.w_sword },
			{ m_axeId, m_banks.w_axe },
			{ m_daggerId, m_banks.w_dagger },
			{ m_katanaId, m_banks.w_katana },
			{ m_tridentId, m_banks.w_trident },
			{ m_hammerId, m_banks.w_hammer },
			{ m_poisonBlowgunId, m_banks.w_poison_blowgun },
			{ m_clubId, m_banks.w_club },
			{ m_scytheId, m_banks.w_scythe },
			{ m_spearId, m_banks.w_spear },
			{ m_kunaiId, m_banks.w_kunai },
			{ m_boomerangId, m_banks.w_boomerang },
			{ m_shurikenId, m_banks.w_shuriken },
			{ m_bigSwordId, m_banks.w_big_sword },
			{ m_electricStaffId, m_banks.w_electric_staff },
			{ m_explosionStaffId, m_banks.w_explosion_staff },
			{ m_frostStaffId, m_banks.w_frost_staff },
			{ m_mahoragaId, m_banks.w_mahoraga },
			{ m_vampireKnifeId, m_banks.w_vampire_knife },
			{ m_shieldId, m_banks.w_shield },
			{ m_wrenchId, m_banks.w_wrench },
			{ m_glaiveId, m_banks.w_glaive },
			{ m_sickleRId, m_banks.w_sickle_r },
			{ m_flaskId, m_banks.w_flask },
			{ m_lanceId, m_banks.w_lance },
			{ m_characterIdUnarmed, m_banks.w_unarmed },
		};

		for ( const auto& p : pairs )
		{
			if ( p.id == weapon && p.bank >= 0 )
				return p.bank;
		}
		// Fallbacks : coup "unarmed" sinon "melee_hit" générique, sinon "rebound".
		if ( m_banks.w_unarmed >= 0 )
			return m_banks.w_unarmed;
		if ( m_banks.melee_hit >= 0 )
			return m_banks.melee_hit;
		return m_banks.rebound;
	}
	float ComputeAutoZoom() const
	{
		const float ref = std::max( m_arenaHalfWidthParam, m_arenaHalfHeightParam );
		return m_cameraZoomPerUnit * ref;
	}
	static float ZoomForPreset( int idx )
	{
		switch ( std::clamp( idx, 0, 2 ) )
		{
			default:
			case 0:
				return 20.0f;
			case 1:
				return 40.0f;
			case 2:
				return 80.0f;
		}
	}

	// ========= Spear helpers =========

	inline float SnapTo( float x, float step )
	{
		return std::round( x / step ) * step;
	}

	inline float SpearPivotB2ForLevel( int level )
	{
		level = std::clamp( level, 1, kPivotAnchorLevel );
		const float t = float( level - 1 ) / float( kPivotAnchorLevel - 1 );
		constexpr float gamma = 0.98f;
		const float eased = std::pow( t, gamma );

		float p = kPivotL1 + ( kPivotL10 - kPivotL1 ) * eased;
		return SnapTo( p, kPivotSnapStep );
	}

	inline void BuildSpearFromSprite( const PixelArtColor* art, float px )
	{
		if ( !art || !B2_IS_NON_NULL( m_spearId ) )
			return;

		// -- Purge TOTALE (shapes + tables auxiliaires)
		{
			b2ShapeId shapes[1024];
			for ( ;; )
			{
				int n = b2Body_GetShapes( m_spearId, shapes, (int)std::size( shapes ) );
				if ( n <= 0 )
					break;
				for ( int i = 0; i < n; ++i )
				{
					const uint64_t k = b2StoreShapeId( shapes[i] );
					m_shapeToCharacter.erase( k );
					m_shapeBaseColor.erase( k );
					b2DestroyShape( shapes[i], /*wakeBodies=*/false );
				}
			}
		}

		// -- On NE reset PAS le cooldown : on vide seulement les overlaps actifs pour cet "attacker"
		{
			const uint64_t atk = b2StoreBodyId( m_spearId );
			std::vector<PairKey> toErase;
			toErase.reserve( m_pairOverlap.size() );
			for ( const auto& kv : m_pairOverlap )
				if ( kv.first.attacker == atk )
					toErase.push_back( kv.first );
			for ( const PairKey& k : toErase )
				m_pairOverlap.erase( k );
		}

		// -- Reconstruire depuis le sprite
		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );

		for ( int y = 0; y < art->height; ++y )
		{
			for ( int x = 0; x < art->width; ++x )
			{
				const PixelPhysicsType t = physMap[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = art->at( x, y );

				const float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				const float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				const b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( t == Pixel_Sensor )
				{
					// Sensors spear : WEAPON sensor avec events ACTIFS (la SKIN en face a aussi events)
					sd.isSensor = true;
					sd.enableSensorEvents = true; // FIX: réactivé pour que le lvl1 déclenche
					sd.enableContactEvents = false;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_SKIN; // ne déclenche que sur la SKIN
					b2ShapeId s = b2CreatePolygonShape( m_spearId, &sd, &box );
					m_shapeBaseColor[b2StoreShapeId( s )] = sd.material.customColor;
				}
				else
				{
					// Pixels solides : WEAPON physique (contacts classiques), pas d’events capteurs
					sd.isSensor = false;
					sd.enableSensorEvents = false; // FIX: inutile sur du solide
					sd.enableContactEvents = true; // armes↔armes / proj↔arme / skin (sensor) ↔ solide
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					b2ShapeId s = b2CreatePolygonShape( m_spearId, &sd, &box );
					m_shapeBaseColor[b2StoreShapeId( s )] = sd.material.customColor;
				}
			}
		}

		

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdSpear;
		jd.base.bodyIdB = m_spearId;

		const float p = SpearPivotB2ForLevel( m_spearLevel ); // distance POSITIVE
		jd.base.localFrameB.p = { -p, -p };					  // offset local (signe négatif)
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_spearJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_spearId )] = m_spearJointId;

		// CCD optionnel
		b2Body_SetBullet( m_spearId, false );

		// -- Réveille l’owner + l’arme pour garantir de nouveaux BeginTouch
		if ( B2_IS_NON_NULL( m_characterIdSpear ) )
			b2Body_SetAwake( m_characterIdSpear, true );
		b2Body_SetAwake( m_spearId, true );
	}

	b2Vec2 ComputePixelLocal( const PixelArtColor* art, float pixelSz, int px, int py )
	{
		float lx = ( float( px ) + 0.5f - art->width * 0.5f ) * pixelSz;
		float ly = ( art->height * 0.5f - ( float( py ) + 0.5f ) ) * pixelSz;
		return { lx, ly };
	}

	// ========= État (reprend tes membres existants) =========

	// region Audio
	AudioManager m_audioManager;
	// endregion

	// region Offsets pixel TIP/TAIL (16x16) pixel art offsets
	static constexpr int ARROW_TIP_X = 15, ARROW_TIP_Y = 0;
	static constexpr int ARROW_TAIL_X = 8, ARROW_TAIL_Y = 7;

	static constexpr int FIREWORK_TIP_X = 14, FIREWORK_TIP_Y = 1;
	static constexpr int FIREWORK_TAIL_X = 3, FIREWORK_TAIL_Y = 12;

	static constexpr int VAMPIRE_KNIFE_TIP_X = 10, VAMPIRE_KNIFE_TIP_Y = 5;
	static constexpr int VAMPIRE_KNIFE_TAIL_X = 8, VAMPIRE_KNIFE_TAIL_Y = 7;

	static constexpr int SHURIKEN_TIP_X = 15, SHURIKEN_TIP_Y = 0;
	static constexpr int SHURIKEN_TAIL_X = 7, SHURIKEN_TAIL_Y = 8;

	// ⬇️ MODIF: frost_* → FROST_*
	static constexpr int FROST_TIP_X = 15, FROST_TIP_Y = 0;
	static constexpr int FROST_TAIL_X = 14, FROST_TAIL_Y = 1;

	static constexpr int EXPLOSION_TIP_X = 15, EXPLOSION_TIP_Y = 0;
	static constexpr int EXPLOSION_TAIL_X = 14, EXPLOSION_TAIL_Y = 1;

	// ⬇️ MODIF: electric_staff_* → ELECTRIC_STAFF_*
	static constexpr int ELECTRIC_STAFF_TIP_X = 15, ELECTRIC_STAFF_TIP_Y = 0;
	static constexpr int ELECTRIC_STAFF_TAIL_X = 14, ELECTRIC_STAFF_TAIL_Y = 1;

	static constexpr int POISON_DART_TIP_X = 15, POISON_DART_TIP_Y = 0;
	static constexpr int POISON_DART_TAIL_X = 14, POISON_DART_TAIL_Y = 1;

	// ⬇️ Nouveau: FLASK
	static constexpr int FLASK_TIP_X = 15, FLASK_TIP_Y = 0;
	static constexpr int FLASK_TAIL_X = 8, FLASK_TAIL_Y = 7;

	// endregion

	// region FX / statuts (freeze, slashs, explosions)
	std::unordered_map<uint64_t, double> m_electricStaffFreezeDuration;
	std::vector<FreezeData> m_activeFreezes;

	// Durée par défaut du hit-freeze côté "projectiles"
	static constexpr double kProjectileHitFreezeDefault = 0.05; // 50 ms (à ton goût)
	double m_projectileHitFreeze = kProjectileHitFreezeDefault;

	std::unordered_map<uint64_t, std::pair<double, int>> m_slashBuildUp;
	std::unordered_map<uint64_t, double> m_slashLastTick;
	std::vector<SlashLine> m_activeSlashLines;

	std::vector<ExplosionAnim> m_activeExplosions;
	// endregion

	// region Gameplay – états/mappings généraux

	// par paire (victime, attaquant) -> nb de overlaps shape↔shape actifs
	std::unordered_map<PairKey, int, PairKeyHash, PairKeyEq> m_pairOverlap;
	std::unordered_map<PairKeySym, double, PairKeySymHash, PairKeySymEq> m_lastPairSwitchTime;
	std::unordered_map<PairKey, double, PairKeyHash, PairKeyEq> m_damageCooldown;

	std::unordered_set<uint64_t> m_flipLatched;

	std::unordered_map<uint64_t, int> m_characterHP;
	std::unordered_map<uint64_t, int> m_weaponDamage;
	std::unordered_map<uint64_t, b2ShapeId> m_characterSkinShape;

	std::unordered_map<uint64_t, b2BodyId> m_weaponOwner;
	std::unordered_map<uint64_t, b2BodyId> m_characterWeapon;
	std::unordered_map<uint64_t, b2JointId> m_weaponToJoint;

	std::unordered_map<uint64_t, double> m_lastHitBlinkTime;
	std::unordered_map<uint64_t, uint64_t> m_shapeToCharacter;

	std::unordered_map<b2BodyId, double, BodyIdHash> m_projectilesToDestroyMap;

	std::unordered_map<uint64_t, std::pair<double, int>> m_poisonBuildUp;
	std::unordered_map<uint64_t, uint32_t> m_shapeBaseColor;

	std::unordered_set<b2BodyId, BodyIdHash> m_sfxSeenProjectiles;
	std::unordered_map<uint64_t, double> m_nextReboundSoundAllowed;
	// endregion

	// region Gameplay – spécifiques armes
	std::unordered_map<b2BodyId, int, BodyIdHash> m_shurikenReboundsLeft;

	std::unordered_map<uint64_t, int> m_vampireKnifeHealCount;
	std::unordered_map<uint64_t, int> m_vampireKnifeTotalHealed;

	std::unordered_map<b2BodyId, double, BodyIdHash> m_turretLastShot;
	std::unordered_map<b2BodyId, b2BodyId, BodyIdHash> m_turretOwner;
	// endregion

	// region Monde / arène / spawn & destructions
	std::vector<b2BodyId> m_arenaWalls;
	std::unordered_map<b2BodyId, b2Vec2, BodyIdHash> m_projectileSpawnPos;
	std::vector<b2BodyId> m_charactersToKill;
	// endregion

	// region Paramètres numériques (floats & config)
	float m_killOuterPadX;
	float m_killOuterPadY;

	float m_gravityYParam = -50.0f;
	float m_cameraZoomPerUnit = 2.0f;
	float m_volume = 100.0f;

	float m_minSpeedAll = 10.0f; //
	float m_killRadius = 35.f;

	float m_arenaHalfWidthParam = 10.0f;
	float m_arenaHalfHeightParam = 10.0f;
	float m_wallHalfThicknessParam = 0.5f;

	float m_killOuterPad = 0.25f;

	float m_explosionRadius = 1.f;
	float m_explosionMagnitude = 1.f;

	float m_killXmin = 0, m_killXmax = 0, m_killYmin = 0, m_killYmax = 0;

	float m_slashFxThicknessMin = 1.0f; // épaisseur minimale (fin de vie)
	float m_slashFxThicknessMax = 6.0f; // épaisseur maximale (début de vie)

	float m_spearPixelSize = kPixelSize;

	float m_boomerangReachA = 1.0f;
	float m_boomerangReachB = 1.0f;
	float m_boomerangAnimSpeed = 1.0f;

	// ── Runtime Unarmed (dérivé de la config par ApplyUnarmedTuning) ──────────
	float m_unarmedMinVel = 0.0f;	  // runtime ← ApplyUnarmedTuning()
	float m_unarmedMinVelCap = 0.0f;  // runtime ← ApplyUnarmedTuning()
	float m_unarmedMinVelGrow = 0.0f; // runtime ← ApplyUnarmedTuning()

	float m_unarmedGhostRadius = 0.0f;	// runtime ← ApplyUnarmedTuning()
	float m_unarmedGhostMinDist = 0.0f; // runtime ← ApplyUnarmedTuning()
	double m_unarmedGhostTTL = 0.0;		// runtime ← ApplyUnarmedTuning()
	int m_unarmedGhostMax = 0;			// runtime ← ApplyUnarmedTuning()

	UnarmedTuning m_unarmedCfg{}; // ← unique source de vérité pour Unarmed

	// ── Policy moteur (sens forcé) ─────────────────────────────────────────────
	bool m_forceMotorSign = true; // si true: signe global imposé
	int m_globalMotorSign = +1;	  // +1 => tjrs positif, -1 => tjrs négatif
	float m_motorSpeedAbs = 4.0f; // |ω| en rad/s (ex-constante 5.0f)

	// On garde la liste des revolute joints "armes" pour (ré)appliquer la policy
	std::vector<b2JointId> m_weaponRevoluteJoints;

	static inline float WithSign( float magnitude, int sign )
	{
		const float m = fabsf( magnitude );
		return sign >= 0 ? +m : -m;
	}

	// Enregistre un joint d'arme et applique le signe immédiatement
	void RegisterWeaponJoint( b2JointId j )
	{
		if ( B2_IS_NULL( j ) )
			return;
		m_weaponRevoluteJoints.push_back( j );
		if ( m_forceMotorSign )
		{
			// On impose immédiatement le signe global
			b2RevoluteJoint_SetMotorSpeed( j, WithSign( m_motorSpeedAbs, m_globalMotorSign ) );
			b2RevoluteJoint_EnableMotor( j, true );
		}
	}

	// Réapplique la policy (utile après un "invert", un changement GUI, etc.)
	void EnforceMotorSignOnAllWeaponMotors()
	{
		for ( const auto& j : m_weaponRevoluteJoints )
		{
			if ( B2_IS_NON_NULL( j ) )
			{
				b2RevoluteJoint_SetMotorSpeed( j, WithSign( m_motorSpeedAbs, m_globalMotorSign ) );
				b2RevoluteJoint_EnableMotor( j, true );
			}
		}
	}

	// endregion

	// region Presets / UI / timings d’effets paremètres globaux
	int m_arenaPreset = 0; // choix arène
	b2BodyId m_killBodyId = b2_nullBodyId;
	ArenaConfig m_arenaCfg;

	int m_selectedCharIdx1 = 0; // sélection personnage
	int m_selectedCharIdx2 = 22;

	double m_daggerHitCooldown = kHitCooldownDagger;

	double m_slashTickInterval = 0.25; // cadence des ticks (plus rapide que poison)
	double m_slashFxDuration = 0.10;   // durée d’affichage d’un trait

	b2Vec2 m_explosionPos = { 0.f, 0.f };
	int m_explosionDamage = 1;
	// endregion

	// region Bodies (monde, armes, personnages) & joints
	b2BodyId m_groundId = b2_nullBodyId;

	b2BodyId m_bowId, m_crossbowId, m_swordId, m_axeId, m_daggerId, m_katanaId, m_tridentId, m_hammerId, m_poisonBlowgunId,
		m_clubId, m_scytheId, m_spearId, m_kunaiId, m_boomerangId, m_shurikenId, m_bigSwordId, m_electricStaffId,
		m_explosionStaffId, m_frostStaffId, m_mahoragaId, m_vampireKnifeId, m_shieldId, m_wrenchId, m_glaiveId, m_sickleRId,
		m_flaskId, m_lanceId;

	b2BodyId m_characterIdBow, m_characterIdCrossbowBow, m_characterIdSword, m_characterIdAxe, m_characterIdDagger,
		m_characterIdKatana, m_characterIdTrident, m_characterIdhammer, m_characterIdPoisonBlowgun, m_characterIdClub,
		m_characterIdScythe, m_characterIdSpear, m_characterIdKunai, m_characterIdBoomerang, m_characterIdShuriken,
		m_characterIdBigSword, m_characterIdElectricStaff, m_characterIdExplosionStaff, m_characterIdFrostStaff,
		m_characterIdMahoraga, m_characterIdVampireKnife, m_characterIdShield, m_characterIdWrench, m_characterIdUnarmed,
		m_characterIdGlaive, m_characterIdSickleR, m_characterIdFlask, m_characterIdLance;

	b2JointId m_bowJointId, m_crossbowJointId, m_swordJointId, m_axeJointId, m_daggerJointId, m_katanaJointId, m_tridentJointId,
		m_hammerJointId, m_poisonBlowgunJointId, m_clubJointId, m_scytheJointId, m_spearJointId, m_kunaiJointId,
		m_boomerangJointId, m_shurikenJointId, m_bigSwordJointId, m_electricStaffJointId, m_explosionStaffJointId,
		m_frostStaffJointId, m_mahoragaJointId, m_vampireKnifeJointId, m_shieldJointId, m_wrenchJointId, m_glaiveJointId,
		m_sickleRJointId, m_flaskJointId, m_lanceJointId;
	// endregion

	// region Collections de projectiles / tourelles
	BodySet m_projectileArrows;
	BodySet m_projectileFireworks;
	BodySet m_projectileVampireKnives;
	BodySet m_projectileShuriken;
	BodySet m_projectileFrost;
	BodySet m_projectileExplosion;
	BodySet m_projectileElectricStaff;
	BodySet m_projectilePoisonDarts;

	BodySet m_turretIds;
	BodySet m_projectileTurrets;

	BodySet m_projectileFlask;

	// endregion

	// region Timers/cadences projectiles
	double m_lastArrowTime = 0., m_lastFireworkTime = 0., m_lastVampireKnifeTime = 0., m_lastShurikenTime = 0.,
		   m_lastElectricStaffTime = 0., m_lastExplosionTime = -10.0, m_lastFrostTime = 0., m_lastPoisonDartTime = 0.,
		   m_lastFlaskTime = 0.;
	// endregion

	// region Compteurs / niveaux / gameplay divers
	int m_shurikenBonusRebounds = 0;
	int m_spearLevel = 1;
	b2Vec2 m_arenaCenter = { 0.f, 0.f };

	ArrowBurst m_arrowBurst;

	int m_crossbowVolleyCount = 10; // nb de projectiles simultanés de l’arbalète

	double m_bowAutoInterval = 1.0;
	double m_hammerLastIncTime = 0.0;
	double m_hammerIncInterval = 1.0;

	int m_katanaSlashStacks = 0;
	int m_scythePoisonStacks = 1;
	int m_poisonDartStacks = 1;

	int m_boomerangHitCount = 0;

	int m_lastUnarmedIncStep = -1;
	double m_unarmedLastT = 0.0;
	// endregion

	// region Ghost trail (Unarmed)
	std::deque<GhostCircle> m_unarmedGhosts;
	b2Vec2 m_unarmedLastGhostPos = { 0.f, 0.f };

	// endregion

	// region Logs / debug
	static std::vector<std::string> g_sensorLogs;
	// endregion

	// region RNG / seeds
	uint32_t m_currentSeedAll = 0;
	uint32_t m_currentSeed1v1 = 0;
	std::mt19937 m_rngAll;
	std::mt19937 m_rng1v1;
	// endregion

	// region Roster
	inline static const std::vector<std::string> kAllCharacters = {
		"Bow",		   "Crossbow",	"Sword",		  "Axe",	   "Dagger",		 "Katana",
		"Trident",	   "Hammer",	"Poison_Blowgun", "Club",	   "Scythe",		 "Spear",
		"Kunai",	   "Boomerang", "Shuriken",		  "Big_Sword", "Electric_Staff", "Explosion_Staff",
		"Frost_Staff", "Mahoraga",	"VampireKnife",	  "Shield",	   "Wrench",		 "Unarmed",
		"Glaive",	   "Sickle_R",	"Flask",		  "Lance"

	};
	// endregion

	// region Flags runtime
	bool m_gravityEnabled = true;
	bool m_hammerJustInverted = false;
	bool m_unarmedGhostInit = false;
	// endregion

	// region Tracking divers
	std::unordered_map<PairKeySym, double, PairKeySymHash, PairKeySymEq> m_weaponPairContactTime;

	int m_stepCounter = 0;
	// endregion

private:
	// ─────────────────────────────────────────────────────────────────────────────
	// Step: réorganisé en phases clairement nommées (aucun changement fonctionnel)
	// ─────────────────────────────────────────────────────────────────────────────
	void Step() override
	{
		++m_stepCounter;
		if ( !WorldReady() )
			return;

		const double now = ImGui::GetTime();

		BeginFrame( now );
		PrePhysicsUpdate( now );

		DoPhysicsStep(); // b2World_Step

		PostPhysicsUpdate( now );

		// La régulation de vitesse se fait ici, pour le prochain step
		MaintainMinVelocityForAll(); // saute déjà les corps gelés

		AnimateAndCompute( now );
		ApplyStatuses( now );
		CleanupAndFinalize( now );
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// UpdateGui: sections pliables, helpers regroupés (aucun changement fonctionnel)
	// ─────────────────────────────────────────────────────────────────────────────

	// ─────────────────────────────────────────────────────────────────────────────
	// Helper: dessine une icône pixel-art centrée horizontalement à une position
	// 'topCenter'. 'targetHeightPx' fixe la hauteur finale en pixels écran.
	// 'dimIfDead' diminue l’alpha si le perso est mort.
	// ─────────────────────────────────────────────────────────────────────────────
	static void DrawPixelArtIcon( ImDrawList* dl, const PixelArtColor* art, ImVec2 topCenter, float targetHeightPx,
								  bool dimIfDead = false )
	{
		if ( !dl || !art || art->width <= 0 || art->height <= 0 )
			return;

		// Échelle pour respecter la hauteur cible
		const float px = targetHeightPx / float( art->height );

		// Point de départ (coin haut-gauche) pour centrer horizontalement
		const float totalW = art->width * px;
		ImVec2 origin( topCenter.x - totalW * 0.5f, topCenter.y );

		// Alpha global si mort (icône “grisée” légère sans filtrage coûteux)
		const int globalA = dimIfDead ? 140 : 255;

		for ( int y = 0; y < art->height; ++y )
		{
			for ( int x = 0; x < art->width; ++x )
			{
				uint32_t rgb = art->at( x, y );		// 0xRRGGBB attendu (comme tu l’utilises pour sd.material.customColor)
				if ( rgb == 0 || rgb == 0xFFFFFFu ) // option: 0 ou blanc pur = fond transparent (si tu le souhaites)
					continue;

				const ImU32 col = IM_COL32( ( rgb >> 16 ) & 255, ( rgb >> 8 ) & 255, ( rgb ) & 255, globalA );

				const float x0 = origin.x + x * px;
				const float y0 = origin.y + y * px;
				dl->AddRectFilled( ImVec2( x0, y0 ), ImVec2( x0 + px, y0 + px ), col );
			}
		}
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Helper: retourne le PixelArtColor* de l’icône en fonction du body de l’arme
	// (mapping simple et explicite vers tes noms d’assets PixelArtColor).
	// Adapte les strings si tes registres ont d’autres noms (“BOW”, “SWORD”, etc.).
	// ─────────────────────────────────────────────────────────────────────────────
	const PixelArtColor* GetIconArtForWeapon( b2BodyId w, b2BodyId m_bowId, b2BodyId m_crossbowId, b2BodyId m_swordId,
											  b2BodyId m_axeId, b2BodyId m_daggerId, b2BodyId m_katanaId, b2BodyId m_tridentId,
											  b2BodyId m_hammerId, b2BodyId m_poisonBlowgunId, b2BodyId m_clubId,
											  b2BodyId m_scytheId, b2BodyId m_spearId, b2BodyId m_kunaiId, b2BodyId m_boomerangId,
											  b2BodyId m_shurikenId, b2BodyId m_bigSwordId, b2BodyId m_electric_staffId,
											  b2BodyId m_explosionStaffId, b2BodyId m_frostStaffId, b2BodyId m_mahoragaId,
											  b2BodyId m_vampireKnifeId, b2BodyId m_shieldId, b2BodyId m_wrenchId,
											  // NEW
											  b2BodyId m_glaiveId, b2BodyId m_sickleRId, b2BodyId m_flaskId )
	{
		auto A = [&]( const char* key ) { return PixelArtColor_GetByName( key ); };

		if ( B2_ID_EQUALS( w, m_bowId ) )
			return A( "BOW" );
		if ( B2_ID_EQUALS( w, m_crossbowId ) )
			return A( "CROSSBOW" );
		if ( B2_ID_EQUALS( w, m_swordId ) )
			return A( "SWORD" );
		if ( B2_ID_EQUALS( w, m_axeId ) )
			return A( "AXE" );
		if ( B2_ID_EQUALS( w, m_daggerId ) )
			return A( "DAGGER" );
		if ( B2_ID_EQUALS( w, m_katanaId ) )
			return A( "KATANA" );
		if ( B2_ID_EQUALS( w, m_tridentId ) )
			return A( "TRIDENT" );
		if ( B2_ID_EQUALS( w, m_hammerId ) )
			return A( "HAMMER" );
		if ( B2_ID_EQUALS( w, m_poisonBlowgunId ) )
			return A( "POISON_BLOWGUN" );
		if ( B2_ID_EQUALS( w, m_clubId ) )
			return A( "CLUB" );
		if ( B2_ID_EQUALS( w, m_scytheId ) )
			return A( "SCYTHE" );
		if ( B2_ID_EQUALS( w, m_spearId ) )
			return A( "SPEAR" );
		if ( B2_ID_EQUALS( w, m_kunaiId ) )
			return A( "KUNAI" );
		if ( B2_ID_EQUALS( w, m_boomerangId ) )
			return A( "BOOMERANG" );
		if ( B2_ID_EQUALS( w, m_shurikenId ) )
			return A( "SHURIKEN" );
		if ( B2_ID_EQUALS( w, m_bigSwordId ) )
			return A( "BIG_SWORD" );
		if ( B2_ID_EQUALS( w, m_electric_staffId ) )
			return A( "ELECTRIC_STAFF" );
		if ( B2_ID_EQUALS( w, m_explosionStaffId ) )
			return A( "EXPLOSION_STAFF" );
		if ( B2_ID_EQUALS( w, m_frostStaffId ) )
			return A( "FROST_STAFF" );
		if ( B2_ID_EQUALS( w, m_mahoragaId ) )
			return A( "MAHORAGA" );
		if ( B2_ID_EQUALS( w, m_vampireKnifeId ) )
			return A( "VAMPIRE_KNIFE" );
		if ( B2_ID_EQUALS( w, m_shieldId ) )
			return A( "SHIELD" );
		if ( B2_ID_EQUALS( w, m_wrenchId ) )
			return A( "WRENCH" );
		// NEW
		if ( B2_ID_EQUALS( w, m_glaiveId ) )
			return A( "GLAIVE" );
		if ( B2_ID_EQUALS( w, m_sickleRId ) )
			return A( "SICKLE_R" );
		if ( B2_ID_EQUALS( w, m_flaskId ) )
			return A( "FLASK" );
		if ( B2_ID_EQUALS( w, m_lanceId ) )
			return A( "LANCE" );

		return nullptr;
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Helper: récupère le body de l’arme d’un perso (tu as déjà une lambda similaire)
	// ─────────────────────────────────────────────────────────────────────────────
	inline b2BodyId GetWeaponBodyForChar_Icon( const std::unordered_map<uint64_t, b2BodyId>& m_characterWeapon, b2BodyId ch )
	{
		if ( !B2_IS_NON_NULL( ch ) )
			return b2_nullBodyId;
		auto it = m_characterWeapon.find( b2StoreBodyId( ch ) );
		return it != m_characterWeapon.end() ? it->second : b2_nullBodyId;
	}

	void UpdateGui() override
	{
		// ── Palette d'armes (couleurs EXACTES alignées avec CreateCharacter*) ──
		const uint32_t W_Sword = b2_colorIndianRed;
		const uint32_t W_Dagger = b2_colorLime;
		const uint32_t W_Scythe = b2_colorBlueViolet;
		const uint32_t W_Spear = b2_colorCyan;
		const uint32_t W_Katana = b2_colorAquamarine;
		const uint32_t W_Shuriken = b2_colorOliveDrab;
		const uint32_t W_Bow = b2_colorYellow;
		const uint32_t W_Shield = b2_colorDarkGoldenRod;
		const uint32_t W_Wrench = b2_colorOrange;
		const uint32_t W_Unarmed = b2_colorLightGray;
		const uint32_t W_Trident = b2_colorNavy;
		const uint32_t W_hammer = b2_colorMediumVioletRed;
		const uint32_t W_PoisonBlowgun = b2_colorGreenYellow;
		const uint32_t W_Club = b2_colorSaddleBrown;
		const uint32_t W_Axe = b2_colorDarkSlateGray;
		const uint32_t W_Kunai = b2_colorIndigo;
		const uint32_t W_Boomerang = b2_colorLightGreen;
		const uint32_t W_ElectricStaff = b2_colorBox2DYellow;
		const uint32_t W_BigSword = b2_colorDarkOrange;
		const uint32_t W_Mahoraga = b2_colorGold;
		const uint32_t W_Crossbow = b2_colorDarkGoldenRod;
		const uint32_t W_VampireKnives = b2_colorCrimson;
		const uint32_t W_FrostStaff = b2_colorAquamarine;
		const uint32_t W_ExplosionStaff = b2_colorRed;
		const uint32_t W_Glaive = b2_colorWheat;
		const uint32_t W_SickleR = b2_colorGhostWhite;
		const uint32_t W_Flask = b2_colorWhite;
		const uint32_t W_Lance = b2_colorKhaki;

		// ── État UI local (persistant via statiques) ────────────────────────────
		static float s_gapVS = 20.f;
		static float s_bottomOffsetY = 20.f;
		static float s_bottomSidePad = 20.f;

		static bool s_timerVisible = true;
		static bool s_timerPaused = false;
		static double s_timerStart = ImGui::GetTime();
		static double s_pauseBegin = 0.0;
		static double s_totalPaused = 0.0;

		static bool s_showMenu = true;

		// Sélecteur de taille de police pour BULLES
		static int s_bubbleFontIdx = 2;			 // 0=Regular, 1=Medium, 2=Large, 3=Big, 4=Max
		static bool s_bubbleFontChanged = false; // pour forcer un "reset" léger

		// ── Helpers (lambda utilitaires) ────────────────────────────────────────
		auto ResetTimer = [&]() {
			s_timerStart = ImGui::GetTime();
			s_totalPaused = 0.0;
			s_timerPaused = false;
		};
		auto RebuildArena = [&]() {
			if ( B2_IS_NON_NULL( m_killBodyId ) )
			{
				b2DestroyBody( m_killBodyId );
				m_killBodyId = b2_nullBodyId;
			}
			if ( B2_IS_NON_NULL( m_groundId ) )
			{
				b2DestroyBody( m_groundId );
				m_groundId = b2_nullBodyId;
			}
			CreateGround();
		};

		// Raccourci clavier : Tab pour (dé)plier le menu
		if ( ImGui::IsKeyPressed( ImGuiKey_Tab ) )
			s_showMenu = !s_showMenu;

		// ── Panneau GUI principal ───────────────────────────────────────────────
		if ( s_showMenu )
		{
			ImGui::Separator();
			ImGui::Text( "Arena & Physics" );

			static const char* kArenaNames[] = { "Arena A", "Arena B", "Arena C" };
			if ( ImGui::Combo( "Arena preset", &m_arenaPreset, kArenaNames, IM_ARRAYSIZE( kArenaNames ) ) )
			{
				ApplyArenaPreset( m_arenaPreset );
				RebuildArena();
			}

			const bool oldEnabled = m_gravityEnabled;
			const float oldGy = m_gravityYParam;
			ImGui::Checkbox( "Enable gravity", &m_gravityEnabled );
			ImGui::BeginDisabled( !m_gravityEnabled );
			ImGui::SliderFloat( "Gravity Y", &m_gravityYParam, -100.0f, 0.0f, "%.1f" );
			ImGui::EndDisabled();
			if ( oldEnabled != m_gravityEnabled || oldGy != m_gravityYParam )
				ApplyGravitySetting();

			// Audio
			ImGui::Separator();
			ImGui::Text( "Audio" );
			{
				float vol = m_volume;
				if ( ImGui::SliderFloat( "Master volume", &vol, 0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp ) )
				{
					m_volume = vol;
					m_audioManager.SetVolume( m_volume );
				}
			}

			// Overlay & HUD
			ImGui::Separator();
			ImGui::Text( "Overlay & HUD" );
			ImGui::SliderFloat( "Top VS gap", &s_gapVS, 0.f, 300.f, "%.0f px" );
			ImGui::Checkbox( "Show overhead HP", &m_showOverheadHP );
			ImGui::SameLine();
			ImGui::Checkbox( "Show overhead bubbles", &m_showOverheadBubbles );

			// >>> Sélecteur de taille pour les bulles
			{
				static const char* kBubbleFontOptions[] = { "Regular", "Medium", "Large", "Big", "Max" };
				int prev = s_bubbleFontIdx;
				ImGui::Combo( "Bubble text size", &s_bubbleFontIdx, kBubbleFontOptions, IM_ARRAYSIZE( kBubbleFontOptions ) );
				if ( prev != s_bubbleFontIdx && ImGui::IsItemDeactivatedAfterEdit() )
				{
					s_bubbleFontChanged = true; // sera traité plus bas (flush caches + mini reset)
					ResetTimer();				// “ça reset” visuellement (chrono)
				}
			}

			ImGui::SliderFloat( "Bottom blocks vertical offset (from ground bottom)", &s_bottomOffsetY, -100.f, 400.f,
								"%.0f px" );
			ImGui::SliderFloat( "Bottom blocks side padding (from ground edges)", &s_bottomSidePad, 0.f, 300.f, "%.0f px" );

			// Timer
			ImGui::Separator();
			ImGui::Text( "Timer (mm:ss)" );
			ImGui::Checkbox( "Show timer", &s_timerVisible );
			ImGui::SameLine();
			if ( ImGui::Button( s_timerPaused ? "Resume" : "Pause" ) )
			{
				if ( !s_timerPaused )
				{
					s_timerPaused = true;
					s_pauseBegin = ImGui::GetTime();
				}
				else
				{
					s_timerPaused = false;
					s_totalPaused += ImGui::GetTime() - s_pauseBegin;
				}
			}
			ImGui::SameLine();
			if ( ImGui::Button( "Reset" ) )
				ResetTimer();

			// Fenêtre “Combat / Spawn”
			ImGui::Begin( "Combat / Spawn" );
			{
				ImGui::Separator();
				ImGui::Text( "1v1 Mode" );

				ImGui::Text( "Current seed: %u", m_currentSeed1v1 );
				static int userSeed1v1 = 0;
				ImGui::InputInt( "Manual seed (1v1)", &userSeed1v1 );
				if ( ImGui::Button( "Restart 1v1 with this seed" ) )
				{
					m_currentSeed1v1 = (uint32_t)userSeed1v1;
					m_rng1v1.seed( m_currentSeed1v1 );
					ClearAllCharacters();
					CreateGround();
					SpawnSelectedCharacters( kAllCharacters[m_selectedCharIdx1], kAllCharacters[m_selectedCharIdx2], m_rng1v1 );
					ResetTimer();
				}
				ImGui::SameLine();
				if ( ImGui::Button( "New random seed (1v1)" ) )
				{
					m_currentSeed1v1 = std::random_device{}();
					m_rng1v1.seed( m_currentSeed1v1 );
					userSeed1v1 = (int)m_currentSeed1v1;
					ClearAllCharacters();
					CreateGround();
					SpawnSelectedCharacters( kAllCharacters[m_selectedCharIdx1], kAllCharacters[m_selectedCharIdx2], m_rng1v1 );
					ResetTimer();
				}

				// Liste “NN — Name” pour combos
				ImGui::Separator();
				auto BuildItemsZ = [&]() -> std::string {
					std::string itemsZ;
					itemsZ.reserve( kAllCharacters.size() * 16 );
					for ( int i = 0; i < (int)kAllCharacters.size(); ++i )
					{
						char buf[128];
						std::snprintf( buf, sizeof( buf ), "%02d — %s", i, kAllCharacters[(size_t)i].c_str() );
						itemsZ.append( buf );
						itemsZ.push_back( '\0' );
					}
					itemsZ.push_back( '\0' );
					return itemsZ;
				};
				static std::string s_itemsZ;
				static int s_itemsZCount = -1;
				if ( s_itemsZCount != (int)kAllCharacters.size() )
				{
					s_itemsZ = BuildItemsZ();
					s_itemsZCount = (int)kAllCharacters.size();
				}
				ImGui::Combo( "Character 1", &m_selectedCharIdx1, s_itemsZ.c_str() );
				ImGui::Combo( "Character 2", &m_selectedCharIdx2, s_itemsZ.c_str() );
				if ( m_selectedCharIdx1 < 0 || m_selectedCharIdx1 >= (int)kAllCharacters.size() )
					m_selectedCharIdx1 = 0;
				if ( m_selectedCharIdx2 < 0 || m_selectedCharIdx2 >= (int)kAllCharacters.size() )
					m_selectedCharIdx2 = 1;

				// Flask preset
				ImGui::Separator();
				ImGui::Text( "Flask options" );
				{
					int idx = (int)m_flaskColorPreset;
					ImGui::Combo( "Flask color", &idx, kFlaskColorNames, (int)FlaskColor::COUNT );
					m_flaskColorPreset = (FlaskColor)idx;
				}

				if ( ImGui::Button( "Spawn 1v1" ) )
				{
					ClearAllCharacters();
					CreateGround();
					SpawnSelectedCharacters( kAllCharacters[m_selectedCharIdx1], kAllCharacters[m_selectedCharIdx2], m_rng1v1 );
					ResetTimer();
				}

				// ALL
				ImGui::Separator();
				ImGui::Text( "ALL Characters Mode" );
				ImGui::Text( "Current seed: %u", m_currentSeedAll );
				static int userSeedAll = 0;
				ImGui::InputInt( "Manual seed (ALL)", &userSeedAll );
				if ( ImGui::Button( "Restart ALL with this seed" ) )
				{
					m_currentSeedAll = (uint32_t)userSeedAll;
					m_rngAll.seed( m_currentSeedAll );
					ClearAllCharacters();
					CreateGround();
					SpawnAllCharacters( m_rngAll );
					ResetTimer();
				}
				ImGui::SameLine();
				if ( ImGui::Button( "New random seed (ALL)" ) )
				{
					m_currentSeedAll = std::random_device{}();
					m_rngAll.seed( m_currentSeedAll );
					userSeedAll = (int)m_currentSeedAll;
					ClearAllCharacters();
					CreateGround();
					SpawnAllCharacters( m_rngAll );
					ResetTimer();
				}
				if ( ImGui::Button( "Spawn ALL Characters" ) )
				{
					ClearAllCharacters();
					CreateGround();
					SpawnAllCharacters( m_rngAll );
					ResetTimer();
				}
			}
			ImGui::End();
		}

		// ── Préparation dessin (fonts + drawlist + helpers) ─────────────────────
		ImFont* fontRegular = m_context->regularFont ? m_context->regularFont : ImGui::GetFont();
		ImFont* fontMedium = m_context->mediumFont ? m_context->mediumFont : fontRegular;
		ImFont* fontLarge = m_context->largeFont ? m_context->largeFont : fontMedium;

		// Pas de big/max dans SampleContext → on retombe sur large
		ImFont* fontBig = fontLarge;
		ImFont* fontMax = fontLarge;


		// Pick font for BUBBLES
		auto PickBubbleFont = [&]() -> ImFont* {
			switch ( s_bubbleFontIdx )
			{
				case 0:
					return fontRegular;
				case 1:
					return fontMedium;
				case 2:
					return fontLarge;
				case 3:
					return fontBig;
				case 4:
					return fontMax;
				default:
					return fontLarge;
			}
		};

		ImDrawList* dl = ImGui::GetForegroundDrawList();
		auto OutlinePxFor = []( float size ) -> int { return (int)std::clamp( size / 18.f, 1.f, 3.f ); };
		auto DrawTextOutlined = [&]( ImFont* f, float size, ImVec2 p, ImU32 mainCol, const char* txt, int px = -1 ) {
			const ImU32 outline = IM_COL32( 0, 0, 0, 255 );
			if ( px < 0 )
				px = OutlinePxFor( size );
			if ( px > 0 )
			{
				dl->AddText( f, size, ImVec2( p.x - px, p.y ), outline, txt );
				dl->AddText( f, size, ImVec2( p.x + px, p.y ), outline, txt );
				dl->AddText( f, size, ImVec2( p.x, p.y - px ), outline, txt );
				dl->AddText( f, size, ImVec2( p.x, p.y + px ), outline, txt );
			}
			dl->AddText( f, size, p, mainCol, txt );
		};
		auto BodyValid = [&]( b2BodyId id ) -> bool { return B2_IS_NON_NULL( id ) && b2Body_IsValid( id ); };
		auto JointValid = [&]( b2JointId id ) -> bool { return B2_IS_NON_NULL( id ) && b2Joint_IsValid( id ); };

		auto IsCharacterDead = [&]( b2BodyId ch ) -> bool {
			if ( !BodyValid( ch ) )
				return true;
			if ( auto it = m_characterHP.find( b2StoreBodyId( ch ) ); it != m_characterHP.end() )
				return it->second <= 0;
			return false;
		};
		auto GrayIfDead = [&]( ImU32 col, b2BodyId ch ) -> ImU32 {
			return IsCharacterDead( ch ) ? IM_COL32( 160, 160, 160, 255 ) : col;
		};
		auto ColFromHex = []( uint32_t rgb, uint8_t a = 255 ) -> ImU32 {
			return IM_COL32( ( rgb >> 16 ) & 255, ( rgb >> 8 ) & 255, rgb & 255, a );
		};

		// ========================================================================
		// Couleurs PERSONNAGES (identiques à CreateCharacter*, casse tolérée)
		// ========================================================================
		static std::unordered_map<std::string, uint32_t> s_charColorHex = {
			{ "Bow", b2_colorYellow },
			{ "BOW", b2_colorYellow },
			{ "Crossbow", b2_colorDarkGoldenRod },
			{ "CROSSBOW", b2_colorDarkGoldenRod },
			{ "VampireKnife", b2_colorCrimson },
			{ "VAMPIRE_KNIFE", b2_colorCrimson },
			{ "Vampire Knife", b2_colorCrimson },
			{ "VAMPIRE KNIFE", b2_colorCrimson },
			{ "Sword", b2_colorIndianRed },
			{ "SWORD", b2_colorIndianRed },
			{ "Dagger", b2_colorLime },
			{ "DAGGER", b2_colorLime },
			{ "Axe", b2_colorDarkSlateGray },
			{ "AXE", b2_colorDarkSlateGray },
			{ "Katana", b2_colorAquamarine },
			{ "KATANA", b2_colorAquamarine },
			{ "Trident", b2_colorNavy },
			{ "TRIDENT", b2_colorNavy },
			{ "Spear", b2_colorCyan },
			{ "SPEAR", b2_colorCyan },
			{ "Mahoraga", b2_colorGold },
			{ "MAHORAGA", b2_colorGold },
			{ "Scythe", b2_colorBlueViolet },
			{ "SCYTHE", b2_colorBlueViolet },
			{ "BigSword", b2_colorDarkOrange },
			{ "BIG_SWORD", b2_colorDarkOrange },
			{ "Big Sword", b2_colorDarkOrange },
			{ "BIG SWORD", b2_colorDarkOrange },
			{ "Hammer", b2_colorMediumVioletRed },
			{ "HAMMER", b2_colorMediumVioletRed },
			{ "PoisonBlowgun", b2_colorGreenYellow },
			{ "POISON_BLOWGUN", b2_colorGreenYellow },
			{ "Poison Blowgun", b2_colorGreenYellow },
			{ "POISON BLOWGUN", b2_colorGreenYellow },
			{ "Club", b2_colorSaddleBrown },
			{ "CLUB", b2_colorSaddleBrown },
			{ "Kunai", b2_colorIndigo },
			{ "KUNAI", b2_colorIndigo },
			{ "Boomerang", b2_colorLightGreen },
			{ "BOOMERANG", b2_colorLightGreen },
			{ "Shuriken", b2_colorOliveDrab },
			{ "SHURIKEN", b2_colorOliveDrab },
			{ "ElectricStaff", b2_colorBox2DYellow },
			{ "ELECTRIC_STAFF", b2_colorBox2DYellow },
			{ "Electric Staff", b2_colorBox2DYellow },
			{ "ELECTRIC STAFF", b2_colorBox2DYellow },
			{ "ExplosionStaff", b2_colorRed },
			{ "EXPLOSION_STAFF", b2_colorRed },
			{ "Explosion Staff", b2_colorRed },
			{ "EXPLOSION STAFF", b2_colorRed },
			{ "FrostStaff", b2_colorAquamarine },
			{ "FROST_STAFF", b2_colorAquamarine },
			{ "Frost Staff", b2_colorAquamarine },
			{ "FROST STAFF", b2_colorAquamarine },
			{ "Shield", b2_colorDarkGoldenRod },
			{ "SHIELD", b2_colorDarkGoldenRod },
			{ "Wrench", b2_colorOrange },
			{ "WRENCH", b2_colorOrange },
			{ "Unarmed", b2_colorLightGray },
			{ "UNARMED", b2_colorLightGray },
			{ "Lance", W_Lance },
			{ "LANCE", W_Lance },
		};

		auto ToUpperASCII = []( std::string s ) {
			for ( auto& c : s )
				c = (char)std::toupper( (unsigned char)c );
			return s;
		};
		auto CharTintByName = [&]( const std::string& name ) -> ImU32 {
			if ( auto it = s_charColorHex.find( name ); it != s_charColorHex.end() )
				return ColFromHex( it->second );
			std::string up = ToUpperASCII( name );
			if ( auto it = s_charColorHex.find( up ); it != s_charColorHex.end() )
				return ColFromHex( it->second );
			return IM_COL32( 255, 255, 255, 255 );
		};

		// Typo HUD
		const ImU32 colH2Label = IM_COL32( 255, 255, 255, 255 );
		const ImU32 colH3Text = IM_COL32( 255, 255, 255, 255 );
		const ImU32 colH4Text = IM_COL32( 255, 255, 255, 255 );

		const float fsH1 = fontLarge->FontSize;
		const float fsH2 = fontMedium->FontSize;
		const float fsH3 = fontMedium->FontSize;
		const float fsH4 = fontMedium->FontSize;
		const float gap = 0.f;

		// Helpers armes ↔ corps/joints
		auto GetWeaponBodyForChar = [&]( b2BodyId ch ) -> b2BodyId {
			if ( !BodyValid( ch ) )
				return b2_nullBodyId;
			if ( auto it = m_characterWeapon.find( b2StoreBodyId( ch ) ); it != m_characterWeapon.end() )
				return it->second;
			return b2_nullBodyId;
		};
		auto GetWeaponName = [&]( b2BodyId w ) -> const char* {
			if ( !BodyValid( w ) )
				return "Unarmed";
			if ( w == m_bowId )
				return "Bow";
			if ( w == m_crossbowId )
				return "Crossbow";
			if ( w == m_swordId )
				return "Sword";
			if ( w == m_axeId )
				return "Axe";
			if ( w == m_daggerId )
				return "Dagger";
			if ( w == m_katanaId )
				return "Katana";
			if ( w == m_tridentId )
				return "Trident";
			if ( w == m_hammerId )
				return "Hammer";
			if ( w == m_poisonBlowgunId )
				return "Poison Blowgun";
			if ( w == m_clubId )
				return "Club";
			if ( w == m_scytheId )
				return "Scythe";
			if ( w == m_spearId )
				return "Spear";
			if ( w == m_kunaiId )
				return "Kunai";
			if ( w == m_boomerangId )
				return "Boomerang";
			if ( w == m_shurikenId )
				return "Shuriken";
			if ( w == m_bigSwordId )
				return "Big Sword";
			if ( w == m_electricStaffId )
				return "Electric Staff";
			if ( w == m_explosionStaffId )
				return "Explosion Staff";
			if ( w == m_frostStaffId )
				return "Frost Staff";
			if ( w == m_mahoragaId )
				return "Mahoraga";
			if ( w == m_vampireKnifeId )
				return "Vampire Knife";
			if ( w == m_shieldId )
				return "Shield";
			if ( w == m_wrenchId )
				return "Wrench";
			if ( w == m_glaiveId )
				return "Glaive";
			if ( w == m_sickleRId )
				return "Sickle_R";
			if ( w == m_flaskId )
				return "Flask";
			return "Weapon";
		};
		auto GetAnyMotorSpeedAbs = [&]( b2BodyId prefer, b2BodyId fallback ) -> float {
			auto readFrom = [&]( b2BodyId b ) -> float {
				if ( !BodyValid( b ) )
					return 0.f;
				b2JointId joints[16];
				int n = b2Body_GetJoints( b, joints, 16 );
				for ( int i = 0; i < n; ++i )
				{
					b2JointId j = joints[i];
					if ( !JointValid( j ) )
						continue;
					if ( b2Joint_GetType( j ) == b2_revoluteJoint )
						return std::abs( b2RevoluteJoint_GetMotorSpeed( j ) );
				}
				return 0.f;
			};
			float s = readFrom( prefer );
			return s == 0.f ? readFrom( fallback ) : s;
		};

		const ImU32 kDamageDigitsBase = IM_COL32( 255, 255, 255, 255 );
		const ImU32 kUnarmedDigits = ColFromHex( W_Unarmed );
		auto WeaponDigitColor = [&]( b2BodyId w, uint8_t a = 255 ) -> ImU32 {
			if ( !BodyValid( w ) )
				return kUnarmedDigits;
			if ( w == m_swordId )
				return ColFromHex( W_Sword, a );
			if ( w == m_daggerId )
				return ColFromHex( W_Dagger, a );
			if ( w == m_scytheId )
				return ColFromHex( W_Scythe, a );
			if ( w == m_spearId )
				return ColFromHex( W_Spear, a );
			if ( w == m_katanaId )
				return ColFromHex( W_Katana, a );
			if ( w == m_shurikenId )
				return ColFromHex( W_Shuriken, a );
			if ( w == m_bowId )
				return ColFromHex( W_Bow, a );
			if ( w == m_shieldId )
				return ColFromHex( W_Shield, a );
			if ( w == m_wrenchId )
				return ColFromHex( W_Wrench, a );
			if ( w == m_tridentId )
				return ColFromHex( W_Trident, a );
			if ( w == m_hammerId )
				return ColFromHex( W_hammer, a );
			if ( w == m_poisonBlowgunId )
				return ColFromHex( W_PoisonBlowgun, a );
			if ( w == m_clubId )
				return ColFromHex( W_Club, a );
			if ( w == m_axeId )
				return ColFromHex( W_Axe, a );
			if ( w == m_kunaiId )
				return ColFromHex( W_Kunai, a );
			if ( w == m_boomerangId )
				return ColFromHex( W_Boomerang, a );
			if ( w == m_electricStaffId )
				return ColFromHex( W_ElectricStaff, a );
			if ( w == m_bigSwordId )
				return ColFromHex( W_BigSword, a );
			if ( w == m_mahoragaId )
				return ColFromHex( W_Mahoraga, a );
			if ( w == m_crossbowId )
				return ColFromHex( W_Crossbow, a );
			if ( w == m_vampireKnifeId )
				return ColFromHex( W_VampireKnives, a );
			if ( w == m_frostStaffId )
				return ColFromHex( W_FrostStaff, a );
			if ( w == m_explosionStaffId )
				return ColFromHex( W_ExplosionStaff, a );
			if ( w == m_glaiveId )
				return ColFromHex( W_Glaive, a );
			if ( w == m_sickleRId )
				return ColFromHex( W_SickleR, a );
			if ( w == m_flaskId )
				return ColFromHex( W_Flask, a );
			return kDamageDigitsBase;
		};

		auto DigitTint = [&]( const std::string& charName, b2BodyId ch, b2BodyId w, bool isDynamic ) -> ImU32 {
			(void)ch;
			(void)w;
			if ( !isDynamic )
				return IM_COL32( 255, 255, 255, 255 );
			return CharTintByName( charName );
		};

		enum class BubbleSrc
		{
			None,
			H2,
			H3
		};
		auto GetBubbleSrc = [&]( b2BodyId w, bool unarmed ) -> BubbleSrc {
			if ( unarmed )
				return BubbleSrc::H3;
			if ( !BodyValid( w ) )
				return BubbleSrc::None;
			if ( w == m_katanaId || w == m_bowId || w == m_scytheId || w == m_shurikenId || w == m_wrenchId || w == m_spearId ||
				 w == m_vampireKnifeId || w == m_daggerId )
				return BubbleSrc::H3;
			if ( w == m_explosionStaffId )
				return BubbleSrc::H2;
			if ( w == m_poisonBlowgunId || w == m_shieldId || w == m_electricStaffId || w == m_frostStaffId )
				return BubbleSrc::None;
			return BubbleSrc::H2;
		};

		// ── HUD struct + caches ─────────────────────────────────────────────────
		struct HUDLines
		{
			std::string h1, h2, h3, h4;
			ImU32 h1Color = IM_COL32_WHITE;
			ImU32 h1TitleColor = IM_COL32_WHITE;
			ImU32 h2DigitCol = IM_COL32_WHITE;
			ImU32 h3DigitCol = IM_COL32_WHITE;
			ImU32 h3TextCol = IM_COL32_WHITE;
			bool h2Dynamic = false;
			bool h3Dynamic = false;
		};
		static std::unordered_map<std::string, std::string> s_lastWeaponLabelByChar;
		static std::unordered_map<std::string, HUDLines> s_lastHUDByChar;
		static std::unordered_map<std::string, const PixelArtColor*> s_lastWeaponArtByChar;

		// Flush caches si taille bulle vient de changer (mini “reset”)
		if ( s_bubbleFontChanged )
		{
			s_lastWeaponLabelByChar.clear();
			s_lastHUDByChar.clear();
			s_lastWeaponArtByChar.clear();
			s_bubbleFontChanged = false;
		}

		auto ComputeHUDLive = [&]( b2BodyId ch ) -> HUDLines {
			HUDLines H;
			if ( !BodyValid( ch ) )
				return H;

			b2BodyId w = GetWeaponBodyForChar( ch );
			const bool unarmed = ( !BodyValid( w ) || w == ch );

			H.h1 = unarmed ? "Unarmed" : GetWeaponName( w );
			H.h1Color = unarmed ? kUnarmedDigits : WeaponDigitColor( w );
			H.h1TitleColor = H.h1Color;

			int dmg = 0;
			if ( BodyValid( w ) )
			{
				if ( auto it = m_weaponDamage.find( b2StoreBodyId( w ) ); it != m_weaponDamage.end() )
					dmg = it->second;
			}
			else
			{
				if ( auto it = m_weaponDamage.find( b2StoreBodyId( ch ) ); it != m_weaponDamage.end() )
					dmg = it->second;
			}
			H.h2 = std::to_string( std::max( 0, dmg ) );
			H.h2DigitCol = IM_COL32( 255, 255, 255, 255 );

			H.h3.clear();
			H.h3DigitCol = IM_COL32( 255, 255, 255, 255 );
			H.h3TextCol = colH3Text;

			float ms = GetAnyMotorSpeedAbs( w, ch );

			// Détails par arme (inchangés)
			if ( w == m_daggerId )
			{
				char b1[64];
				std::snprintf( b1, sizeof b1, "Speed attack: %.2f", ms );
				H.h3 = b1;
				H.h4 = "+ Speed attack ";
				H.h3Dynamic = true;
			}
			else if ( w == m_bowId )
			{
				int rem = m_arrowBurst.active ? std::max( 0, m_arrowBurst.total - m_arrowBurst.shot ) : m_arrowBurst.total;
				H.h3 = "Arrows: " + std::to_string( rem );
				H.h4 = "Arrow +1 on hit";
				H.h3Dynamic = true;
			}
			else if ( w == m_scytheId )
			{
				const int stacksPerHit = std::max( 1, m_scythePoisonStacks );
				H.h3 = "Poison +" + std::to_string( stacksPerHit ) + "/hit | 2/s/stack";
				H.h3Dynamic = true;
			}
			else if ( w == m_wrenchId )
			{
				int tur = 0;
				for ( const auto& t : m_turretIds )
				{
					if ( !BodyValid( t ) )
						continue;
					auto it = m_turretOwner.find( t );
					if ( it != m_turretOwner.end() && B2_IS_NON_NULL( it->second ) &&
						 b2StoreBodyId( it->second ) == b2StoreBodyId( ch ) )
						++tur;
				}
				H.h3 = "Turrets: " + std::to_string( tur );
				H.h4 = "Turret +1 on hit";
				H.h3Dynamic = true;
			}
			else if ( w == m_spearId )
			{
				int L = std::max( 1, m_spearLevel );
				H.h3 = "Level: L" + std::to_string( L );
				H.h4 = "Length & dmg +1 on hit";
				H.h2Dynamic = true;
				H.h3Dynamic = true;
			}
			else if ( w == m_vampireKnifeId )
			{
				int healed = 0;
				if ( auto it = m_vampireKnifeTotalHealed.find( b2StoreBodyId( ch ) ); it != m_vampireKnifeTotalHealed.end() )
					healed = it->second;
				H.h3 = "Healed: " + std::to_string( healed ) + " HP";
				H.h4 = "+lifesteal | +1 dmg /10 heal";
				H.h2Dynamic = true;
				H.h3Dynamic = true;
			}
			else if ( w == m_poisonBlowgunId )
			{
				const int stacks = std::max( 1, m_poisonDartStacks );
				H.h3 = "Poison +" + std::to_string( stacks ) + "/hit | 2/s/stack";
				H.h4 = "Poison dart on hit";
				H.h3Dynamic = true;
			}
			else if ( w == m_shieldId )
			{
				H.h3 = "Reflect: ON";
				H.h4 = "Reflect dmg & effects";
			}
			else if ( w == m_electricStaffId )
			{
				H.h4 = "Freeze stacks on hit";
			}
			else if ( w == m_frostStaffId )
			{
				H.h4 = "+1 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_katanaId )
			{
				const int s = std::max( 1, m_katanaSlashStacks );
				H.h3 = "Slashes: " + std::to_string( s );
				H.h4 = "+1 slash on hit";
				H.h3Dynamic = true;
			}
			else if ( w == m_swordId )
			{
				H.h4 = "+1 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_crossbowId )
			{
				H.h4 = "+1 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_axeId )
			{
				H.h4 = "×1.5 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_tridentId )
			{
				H.h4 = "+1 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_bigSwordId || w == m_kunaiId || w == m_clubId || w == m_mahoragaId )
			{
				H.h4 = "+1 on hit";
				H.h2Dynamic = true;
			}
			else if ( w == m_explosionStaffId )
			{
				H.h2 = std::to_string( std::max( 0, m_explosionDamage ) );
				char buf[128];
				std::snprintf( buf, sizeof buf, "R: %.1f | F: %.1f | D: %d", m_explosionRadius, m_explosionMagnitude,
							   m_explosionDamage );
				H.h3 = buf;
				H.h4 = "Radius/Force/Damage grow on hit";
				H.h2Dynamic = true;
				H.h3Dynamic = true;
			}
			else if ( w == m_hammerId )
			{
				const double interval = std::max( 0.01, (double)m_hammerIncInterval );
				char buf[64];
				std::snprintf( buf, sizeof buf, "Damage +1 / %.1fs", interval );
				H.h3 = buf;
				H.h2Dynamic = true;
			}
			else if ( w == m_boomerangId )
			{
				const uint64_t key = b2StoreBodyId( w );
				if ( auto it = m_boomerangs.find( key ); it != m_boomerangs.end() )
				{
					const BoomerangState& s = it->second;
					char buf[64];
					std::snprintf( buf, sizeof buf, "Range: %.1f×%.1f", s.reachA, s.reachB );
					H.h3 = buf;
					H.h4 = "+range | +1 dmg /3 hits";
					H.h2Dynamic = true;
					H.h3Dynamic = true;
				}
			}
			else if ( unarmed )
			{
				b2Vec2 v = b2Body_GetLinearVelocity( ch );
				float sp = std::sqrt( v.x * v.x + v.y * v.y );
				char b[64];
				std::snprintf( b, sizeof b, "Speed: %.2f", sp );
				H.h3 = b;
				H.h4 = "Damage = Speed/Hit";
				H.h2Dynamic = true;
				H.h3Dynamic = true;
			}
			else if ( w == m_shurikenId )
			{
				const int maxRb = std::max( 1, 1 + m_shurikenBonusRebounds );
				H.h3 = "Rebounds max: " + std::to_string( maxRb );
				H.h4 = "+1 rebound on hit";
				H.h3Dynamic = true;
			}
			else
			{
				char b[64];
				std::snprintf( b, sizeof b, "Motor: %.2f", ms );
				H.h3 = b;
			}
			return H;
		};

		// Sélection courante
		int i1 = std::clamp( m_selectedCharIdx1, 0, (int)kAllCharacters.size() - 1 );
		int i2 = std::clamp( m_selectedCharIdx2, 0, (int)kAllCharacters.size() - 1 );
		const std::string& leftCharName = kAllCharacters[i1];
		const std::string& rightCharName = kAllCharacters[i2];
		b2BodyId leftId = GetCharacterIdByName( leftCharName );
		b2BodyId rightId = GetCharacterIdByName( rightCharName );

		auto GetHUDForChar = [&]( const std::string& charName, b2BodyId ch ) -> HUDLines {
			const bool alive = BodyValid( ch ) && !IsCharacterDead( ch );
			if ( alive )
			{
				HUDLines H = ComputeHUDLive( ch );
				if ( !H.h1.empty() )
					s_lastWeaponLabelByChar[charName] = H.h1;
				s_lastHUDByChar[charName] = H;
				b2BodyId wForTint = GetWeaponBodyForChar( ch );
				bool unarmedLocal = ( !BodyValid( wForTint ) || wForTint == ch );
				ImU32 baseH1 = unarmedLocal ? kUnarmedDigits : WeaponDigitColor( wForTint );
				H.h1TitleColor = GrayIfDead( baseH1, ch );
				H.h2DigitCol = GrayIfDead( DigitTint( charName, ch, wForTint, H.h2Dynamic ), ch );
				H.h3DigitCol = GrayIfDead( DigitTint( charName, ch, wForTint, H.h3Dynamic ), ch );
				H.h3TextCol = IM_COL32( 255, 255, 255, 255 );
				return H;
			}
			HUDLines H = {};
			if ( auto it = s_lastHUDByChar.find( charName ); it != s_lastHUDByChar.end() )
				H = it->second;
			if ( H.h1.empty() )
				H.h1 = ( s_lastWeaponLabelByChar.count( charName ) ? s_lastWeaponLabelByChar[charName] : "Unarmed" );
			H.h1TitleColor = IM_COL32( 160, 160, 160, 255 );
			H.h2DigitCol = IM_COL32( 160, 160, 160, 255 );
			H.h3DigitCol = IM_COL32( 160, 160, 160, 255 );
			H.h3TextCol = IM_COL32( 160, 160, 160, 255 );
			return H;
		};

		// Rendu de chiffres “teintés” multi-lignes + mesure
		auto DrawTintedNumbersML = [&]( ImFont* f, float size, ImVec2 base, const std::string& s, ImU32 normalCol, ImU32 digitCol,
										int outlinePx, bool rightAlign, float lineGap ) -> float {
			struct Seg
			{
				std::string t;
				bool num;
			};
			auto isNumChar = []( char c ) { return ( c >= '0' && c <= '9' ) || c == '+' || c == '-' || c == '.'; };
			if ( outlinePx < 0 )
				outlinePx = OutlinePxFor( size );

			const float advDigit = [&] {
				float a = 0.f;
				const char* k = "0123456789+-.";
				for ( const char* p = k; *p; ++p )
				{
					char one[2] = { *p, 0 };
					a = std::max( a, f->CalcTextSizeA( size, FLT_MAX, 0.f, one ).x );
				}
				return a;
			}();

			int lineCount = 0;
			size_t start = 0;
			while ( true )
			{
				size_t nl = s.find( '\n', start );
				std::string line = ( nl == std::string::npos ) ? s.substr( start ) : s.substr( start, nl - start );

				std::vector<Seg> segs;
				segs.reserve( line.size() );
				std::string cur;
				bool curNum = false, first = true;
				for ( char c : line )
				{
					bool n = isNumChar( c );
					if ( first )
					{
						curNum = n;
						first = false;
					}
					if ( n != curNum )
					{
						segs.push_back( { cur, curNum } );
						cur.clear();
						curNum = n;
					}
					cur.push_back( c );
				}
				if ( !cur.empty() )
					segs.push_back( { cur, curNum } );

				float totalW = 0.f;
				for ( const auto& sg : segs )
					totalW += sg.num ? advDigit * (float)sg.t.size() : f->CalcTextSizeA( size, FLT_MAX, 0.f, sg.t.c_str() ).x;

				ImVec2 p = base;
				if ( rightAlign )
					p.x -= totalW;

				auto drawOutlined = [&]( ImVec2 pos, const char* txt, ImU32 col ) {
					const ImU32 out = IM_COL32( 0, 0, 0, 255 );
					if ( outlinePx > 0 )
					{
						dl->AddText( f, size, ImVec2( pos.x - outlinePx, pos.y ), out, txt );
						dl->AddText( f, size, ImVec2( pos.x + outlinePx, pos.y ), out, txt );
						dl->AddText( f, size, ImVec2( pos.x, pos.y - outlinePx ), out, txt );
						dl->AddText( f, size, ImVec2( pos.x, pos.y + outlinePx ), out, txt );
					}
					dl->AddText( f, size, pos, col, txt );
				};
				auto drawNormal = [&]( const std::string& t, ImU32 col ) {
					drawOutlined( p, t.c_str(), col );
					p.x += f->CalcTextSizeA( size, FLT_MAX, 0.f, t.c_str() ).x;
				};
				auto drawDigitsMono = [&]( const std::string& t, ImU32 col ) {
					for ( char c : t )
					{
						char one[2] = { c, 0 };
						ImVec2 ts = f->CalcTextSizeA( size, FLT_MAX, 0.f, one );
						float ox = ( advDigit - ts.x ) * 0.5f;
						drawOutlined( ImVec2( p.x + ox, p.y ), one, col );
						p.x += advDigit;
					}
				};

				for ( const auto& sg : segs )
					sg.num ? drawDigitsMono( sg.t, digitCol ) : drawNormal( sg.t, normalCol );

				++lineCount;
				if ( nl == std::string::npos )
					break;
				start = nl + 1;
				base.y += size + lineGap;
			}
			return ( lineCount <= 0 ) ? 0.f : ( size * lineCount + lineGap * ( lineCount - 1 ) );
		};

		auto MeasureTintedLineWidth = [&]( ImFont* f, float size, const std::string& s ) -> float {
			auto isNumChar = []( char c ) { return ( c >= '0' && c <= '9' ) || c == '+' || c == '-' || c == '.'; };
			const float advDigit = [&] {
				float a = 0.f;
				const char* k = "0123456789+-.";
				for ( const char* p = k; *p; ++p )
				{
					char one[2] = { *p, 0 };
					a = std::max( a, f->CalcTextSizeA( size, FLT_MAX, 0.f, one ).x );
				}
				return a;
			}();
			std::string line = s.substr( 0, s.find( '\n' ) );
			float totalW = 0.f;
			bool curNum = false, first = true;
			std::string cur;
			for ( char c : line )
			{
				bool n = isNumChar( c );
				if ( first )
				{
					curNum = n;
					first = false;
				}
				if ( n != curNum )
				{
					totalW += curNum ? advDigit * (float)cur.size() : f->CalcTextSizeA( size, FLT_MAX, 0.f, cur.c_str() ).x;
					cur.clear();
					curNum = n;
				}
				cur.push_back( c );
			}
			if ( !cur.empty() )
				totalW += curNum ? advDigit * (float)cur.size() : f->CalcTextSizeA( size, FLT_MAX, 0.f, cur.c_str() ).x;
			return totalW;
		};

		// ── Ancrage sol/toit en pixels ─────────────────────────────────────────
		const float groundLeftW = -m_arenaHalfWidthParam;
		const float groundRightW = +m_arenaHalfWidthParam;
		const float groundBottomW = -m_arenaHalfHeightParam;
		const float groundTopW = +m_arenaHalfHeightParam;

		const float groundLeftX = ConvertWorldToScreen( &m_context->camera, { groundLeftW, 0.f } ).x;
		const float groundRightX = ConvertWorldToScreen( &m_context->camera, { groundRightW, 0.f } ).x;
		const float baseYBottom = ConvertWorldToScreen( &m_context->camera, { 0.f, groundBottomW } ).y;
		const float baseYTop = ConvertWorldToScreen( &m_context->camera, { 0.f, groundTopW } ).y;

		const float yBottomBlocks = baseYBottom + s_bottomOffsetY;
		const float xLeftBlock = groundLeftX + s_bottomSidePad;
		const float xRightBlock = groundRightX - s_bottomSidePad;

		HUDLines LHUD = GetHUDForChar( leftCharName, leftId );
		HUDLines RHUD = GetHUDForChar( rightCharName, rightId );

		// Tag bas-centre
		{
			const char* kTag = "MevenBox2D";
			ImFont* tagFont = fontMedium;
			float fsTag = tagFont->FontSize;
			ImVec2 tSize = tagFont->CalcTextSizeA( fsTag, FLT_MAX, 0.f, kTag );
			const float xCenterGround = 0.5f * ( groundLeftX + groundRightX ) - 0.5f * tSize.x;
			DrawTextOutlined( tagFont, fsTag, ImVec2( xCenterGround, yBottomBlocks ), IM_COL32( 255, 255, 255, 230 ), kTag );
		}

		// Blocs bas (gauche/droite)
		auto DrawBottomLeft = [&]( float xLeft, float y, const HUDLines& H ) {
			DrawTextOutlined( fontBig, fsH1, ImVec2( xLeft, y ), H.h1TitleColor, H.h1.c_str() );
			y += fsH1 + gap;
			float usedH2 = DrawTintedNumbersML( fontLarge, fsH2, ImVec2( xLeft, y ), std::string( "Damage/Hit: " ) + H.h2,
												colH2Label, H.h2DigitCol, -1, false, 2.f );
			y += usedH2 + gap;
			if ( !H.h3.empty() )
			{
				float usedH3 =
					DrawTintedNumbersML( fontLarge, fsH3, ImVec2( xLeft, y ), H.h3, H.h3TextCol, H.h3DigitCol, -1, false, 2.f );
				y += usedH3 + gap;
			}
			if ( !H.h4.empty() )
				DrawTextOutlined( fontLarge, fsH4, ImVec2( xLeft, y ), colH4Text, H.h4.c_str() );
		};
		auto DrawBottomRight = [&]( float xRight, float y, const HUDLines& H ) {
			auto RightPos = [&]( ImFont* f, float size, const std::string& s ) {
				ImVec2 ts = f->CalcTextSizeA( size, FLT_MAX, 0.f, s.c_str() );
				return ImVec2( xRight - ts.x, y );
			};
			ImVec2 p1 = RightPos( fontBig, fsH1, H.h1 );
			DrawTextOutlined( fontBig, fsH1, p1, H.h1TitleColor, H.h1.c_str() );
			y += fsH1 + gap;
			float usedH2 = DrawTintedNumbersML( fontLarge, fsH2, ImVec2( xRight, y ), std::string( "Damage/Hit: " ) + H.h2,
												colH2Label, H.h2DigitCol, -1, true, 2.f );
			y += usedH2 + gap;
			if ( !H.h3.empty() )
			{
				float usedH3 =
					DrawTintedNumbersML( fontLarge, fsH3, ImVec2( xRight, y ), H.h3, H.h3TextCol, H.h3DigitCol, -1, true, 2.f );
				y += usedH3 + gap;
			}
			if ( !H.h4.empty() )
			{
				ImVec2 p4 = RightPos( fontLarge, fsH4, H.h4 );
				DrawTextOutlined( fontLarge, fsH4, p4, colH4Text, H.h4.c_str() );
			}
		};
		DrawBottomLeft( xLeftBlock, yBottomBlocks, LHUD );
		DrawBottomRight( xRightBlock, yBottomBlocks, RHUD );

		// Pixel-art mapping + draw
		auto WeaponIdToArt = [&]( b2BodyId w ) -> const PixelArtColor* {
			if ( !BodyValid( w ) )
				return nullptr;
			if ( w == m_bowId )
				return PixelArtColor_GetByName( "BOW" );
			if ( w == m_crossbowId )
				return PixelArtColor_GetByName( "CROSSBOW" );
			if ( w == m_swordId )
				return PixelArtColor_GetByName( "SWORD" );
			if ( w == m_axeId )
				return PixelArtColor_GetByName( "AXE" );
			if ( w == m_daggerId )
				return PixelArtColor_GetByName( "DAGGER" );
			if ( w == m_katanaId )
				return PixelArtColor_GetByName( "KATANA" );
			if ( w == m_tridentId )
				return PixelArtColor_GetByName( "TRIDENT" );
			if ( w == m_hammerId )
				return PixelArtColor_GetByName( "HAMMER" );
			if ( w == m_poisonBlowgunId )
				return PixelArtColor_GetByName( "POISON_BLOWGUN" );
			if ( w == m_clubId )
				return PixelArtColor_GetByName( "CLUB" );
			if ( w == m_scytheId )
				return PixelArtColor_GetByName( "SCYTHE" );
			if ( w == m_spearId )
				return PixelArtColor_GetByName( "SPEAR_LVL1" );
			if ( w == m_kunaiId )
				return PixelArtColor_GetByName( "KUNAI" );
			if ( w == m_boomerangId )
				return PixelArtColor_GetByName( "BOOMERANG" );
			if ( w == m_shurikenId )
				return PixelArtColor_GetByName( "SHURIKEN" );
			if ( w == m_bigSwordId )
				return PixelArtColor_GetByName( "BIG_SWORD" );
			if ( w == m_electricStaffId )
				return PixelArtColor_GetByName( "ELECTRIC_STAFF" );
			if ( w == m_explosionStaffId )
				return PixelArtColor_GetByName( "EXPLOSION_STAFF" );
			if ( w == m_frostStaffId )
				return PixelArtColor_GetByName( "FROST_STAFF" );
			if ( w == m_mahoragaId )
				return PixelArtColor_GetByName( "MAHORAGA" );
			if ( w == m_vampireKnifeId )
				return PixelArtColor_GetByName( "VAMPIRE_KNIFE" );
			if ( w == m_shieldId )
				return PixelArtColor_GetByName( "SHIELD" );
			if ( w == m_wrenchId )
				return PixelArtColor_GetByName( "WRENCH" );
			if ( w == m_glaiveId )
				return PixelArtColor_GetByName( "GLAIVE" );
			if ( w == m_sickleRId )
				return PixelArtColor_GetByName( "SICKLE_R" );
			if ( w == m_flaskId )
				return PixelArtColor_GetByName( "FLASK" );
			return nullptr;
		};
		auto DrawPixelArtIcon = [&]( ImDrawList* dl_, const PixelArtColor* art, ImVec2 topCenter, float iconHeightPx, bool dim ) {
			if ( !art )
				return;
			const float scale = iconHeightPx / (float)art->height;
			const float iconWidth = (float)art->width * scale;
			const ImVec2 topLeft = { topCenter.x - iconWidth * 0.5f, topCenter.y };
			std::vector<PixelPhysicsType> map;
			ComputePixelPhysicsMap( *art, map );
			for ( int y = 0; y < art->height; ++y )
				for ( int x = 0; x < art->width; ++x )
				{
					if ( map[y * art->width + x] == Pixel_Void )
						continue;
					uint32_t rgb = art->at( x, y );
					unsigned r = ( rgb >> 16 ) & 255, g = ( rgb >> 8 ) & 255, b = rgb & 255;
					if ( dim )
					{
						r = (unsigned)std::round( std::clamp( (float)r * 0.6f, 0.f, 255.f ) );
						g = (unsigned)std::round( std::clamp( (float)g * 0.6f, 0.f, 255.f ) );
						b = (unsigned)std::round( std::clamp( (float)b * 0.6f, 0.f, 255.f ) );
					}
					ImU32 col = IM_COL32( (int)r, (int)g, (int)b, dim ? 200 : 255 );
					ImVec2 p0 = { topLeft.x + x * scale, topLeft.y + y * scale };
					ImVec2 p1 = { topLeft.x + ( x + 1 ) * scale, topLeft.y + ( y + 1 ) * scale };
					dl_->AddRectFilled( p0, p1, col );
				}
		};

		// Bandeau VS haut
		{
			std::string leftLabel = LHUD.h1, rightLabel = RHUD.h1;
			ImU32 leftColTop = LHUD.h1TitleColor, rightColTop = RHUD.h1TitleColor;

			const float fsNames = fontBig->FontSize;
			const float fsVS = fontMax->FontSize;

			ImVec2 szL = fontBig->CalcTextSizeA( fsNames, FLT_MAX, 0.f, leftLabel.c_str() );
			ImVec2 szV = fontMax->CalcTextSizeA( fsVS, FLT_MAX, 0.f, "VS" );
			ImVec2 szR = fontBig->CalcTextSizeA( fsNames, FLT_MAX, 0.f, rightLabel.c_str() );

			const float sidePadTop = 32.f;
			const float gapVS = s_gapVS;

			ImVec2 disp = ImGui::GetIO().DisplaySize;
			const float xVS = disp.x * 0.5f - szV.x * 0.5f;
			const float xLeft = std::max( sidePadTop, disp.x * 0.5f - gapVS - szV.x * 0.5f - szL.x );
			const float xRight = std::min( disp.x - sidePadTop - szR.x, disp.x * 0.5f + gapVS + szV.x * 0.5f );

			float topVS = baseYTop - 10.0f - szV.y;
			float midY = topVS + szV.y * 0.5f;

			DrawTextOutlined( fontBig, fsNames, ImVec2( xLeft, midY - szL.y * 0.5f ), leftColTop, leftLabel.c_str() );
			DrawTextOutlined( fontMax, fsVS, ImVec2( xVS, topVS ), IM_COL32( 255, 255, 255, 255 ), "VS" );
			DrawTextOutlined( fontBig, fsNames, ImVec2( xRight, midY - szR.y * 0.5f ), rightColTop, rightLabel.c_str() );

			auto IsCharacterDeadLocal = [&]( b2BodyId ch ) {
				if ( !B2_IS_NON_NULL( ch ) )
					return true;
				auto it = m_characterHP.find( b2StoreBodyId( ch ) );
				return ( it != m_characterHP.end() ) ? ( it->second <= 0 ) : false;
			};

			b2BodyId leftW = GetWeaponBodyForChar( leftId );
			b2BodyId rightW = GetWeaponBodyForChar( rightId );
			const bool leftDead = IsCharacterDeadLocal( leftId );
			const bool rightDead = IsCharacterDeadLocal( rightId );

			const PixelArtColor* artL_live = WeaponIdToArt( leftW );
			const PixelArtColor* artR_live = WeaponIdToArt( rightW );
			if ( !leftDead && artL_live )
				s_lastWeaponArtByChar[leftCharName] = artL_live;
			if ( !rightDead && artR_live )
				s_lastWeaponArtByChar[rightCharName] = artR_live;

			const PixelArtColor* artL =
				leftDead ? ( s_lastWeaponArtByChar.count( leftCharName ) ? s_lastWeaponArtByChar[leftCharName] : artL_live )
						 : artL_live;
			const PixelArtColor* artR =
				rightDead ? ( s_lastWeaponArtByChar.count( rightCharName ) ? s_lastWeaponArtByChar[rightCharName] : artR_live )
						  : artR_live;

			const float iconHeightPx = std::max( 28.0f, fsNames * 1.25f );
			const float gapIconToText = 6.0f;

			ImVec2 lTxtSize = fontBig->CalcTextSizeA( fsNames, FLT_MAX, 0.f, leftLabel.c_str() );
			ImVec2 rTxtSize = fontBig->CalcTextSizeA( fsNames, FLT_MAX, 0.f, rightLabel.c_str() );
			const float leftCenterX = xLeft + lTxtSize.x * 0.5f;
			const float rightCenterX = xRight + rTxtSize.x * 0.5f;
			const float iconBottomY = midY - std::min( lTxtSize.y, rTxtSize.y ) * 0.5f - gapIconToText;
			const float iconTopY = iconBottomY - iconHeightPx;

			if ( artL )
				DrawPixelArtIcon( dl, artL, ImVec2( leftCenterX, iconTopY ), iconHeightPx, leftDead );
			if ( artR )
				DrawPixelArtIcon( dl, artR, ImVec2( rightCenterX, iconTopY ), iconHeightPx, rightDead );
		}

		// ── Overhead : HP + bulle dynamique ────────────────────────────────────
		auto BuildBubbleValue = [&]( const std::string& charName, b2BodyId id ) -> std::pair<std::string, ImU32> {
			if ( !BodyValid( id ) )
				return { "", IM_COL32_WHITE };
			b2BodyId w = GetWeaponBodyForChar( id );
			const bool unarmed = ( !BodyValid( w ) || w == id );
			ImU32 digCol = DigitTint( charName, id, w, true );

			int dmg = 0;
			if ( BodyValid( w ) )
			{
				if ( auto it = m_weaponDamage.find( b2StoreBodyId( w ) ); it != m_weaponDamage.end() )
					dmg = it->second;
			}
			else
			{
				if ( auto it = m_weaponDamage.find( b2StoreBodyId( id ) ); it != m_weaponDamage.end() )
					dmg = it->second;
			}

			BubbleSrc src = GetBubbleSrc( w, unarmed );
			if ( src == BubbleSrc::None )
				return { "", digCol };
			if ( src == BubbleSrc::H2 )
				return { std::to_string( std::max( 0, dmg ) ), digCol };

			if ( unarmed )
			{
				b2Vec2 v = b2Body_GetLinearVelocity( id );
				float sp = std::sqrt( v.x * v.x + v.y * v.y );
				char b[32];
				std::snprintf( b, sizeof b, "%.2f", sp );
				return { b, digCol };
			}
			if ( w == m_daggerId )
			{
				float ms = GetAnyMotorSpeedAbs( w, id );
				char b[48];
				std::snprintf( b, sizeof b, "%.2f", ms );
				return { b, digCol };
			}
			if ( w == m_bowId )
			{
				int rem = m_arrowBurst.active ? std::max( 0, m_arrowBurst.total - m_arrowBurst.shot ) : m_arrowBurst.total;
				return { std::to_string( rem ), digCol };
			}
			if ( w == m_scytheId )
				return { std::to_string( std::max( 1, m_scythePoisonStacks ) ), digCol };
			if ( w == m_shurikenId )
				return { std::to_string( 1 + m_shurikenBonusRebounds ), digCol };
			if ( w == m_wrenchId )
			{
				int tur = 0;
				for ( b2BodyId t : m_turretIds )
				{
					if ( !BodyValid( t ) )
						continue;
					auto it = m_turretOwner.find( t );
					if ( it != m_turretOwner.end() && B2_IS_NON_NULL( it->second ) &&
						 b2StoreBodyId( it->second ) == b2StoreBodyId( id ) )
						++tur;
				}
				return { std::to_string( tur ), digCol };
			}
			if ( w == m_spearId )
				return { std::to_string( std::max( 1, m_spearLevel ) ), digCol };
			if ( w == m_vampireKnifeId )
			{
				int healed = 0;
				if ( auto it = m_vampireKnifeTotalHealed.find( b2StoreBodyId( id ) ); it != m_vampireKnifeTotalHealed.end() )
					healed = it->second;
				return { std::to_string( healed ), digCol };
			}
			if ( w == m_katanaId )
				return { std::to_string( std::max( 1, m_katanaSlashStacks ) ), digCol };
			if ( w == m_poisonBlowgunId )
				return { "1", digCol };

			float ms = GetAnyMotorSpeedAbs( w, id );
			char b[32];
			std::snprintf( b, sizeof b, "%.2f", ms );
			return { b, digCol };
		};

		// ── Dessin des HP + bulle pour tous les personnages présents
		{
			for ( const std::string& name : kAllCharacters )
			{
				b2BodyId id = GetCharacterIdByName( name );
				if ( !BodyValid( id ) )
					continue;

				// >>> Police commune pilotée par "Bubble text size" pour PV **et** bulle
				ImFont* overlayFont = PickBubbleFont();
				const float overlayFS = overlayFont->FontSize;

				int hp = 0;
				if ( auto itHp = m_characterHP.find( b2StoreBodyId( id ) ); itHp != m_characterHP.end() )
					hp = itHp->second;

				b2Vec2 world = b2Body_GetPosition( id );
				b2Vec2 screen = ConvertWorldToScreen( &m_context->camera, world );

				// PV (HP) en surimpression — maintenant avec la police sélectionnée
				char hpTxt[8];
				snprintf( hpTxt, sizeof hpTxt, "%d", hp );
				ImVec2 hpSize = overlayFont->CalcTextSizeA( overlayFS, FLT_MAX, 0.f, hpTxt );
				ImVec2 hpPos( screen.x - hpSize.x * 0.5f, screen.y - hpSize.y * 0.5f );

				if ( m_showOverheadHP )
				{
					// Style identique à avant (texte noir plein). Si tu préfères l’outline blanc :
					// DrawTextOutlined( overlayFont, overlayFS, hpPos, IM_COL32(255,255,255,255), hpTxt );
					dl->AddText( overlayFont, overlayFS, hpPos, IM_COL32( 0, 0, 0, 255 ), hpTxt );
				}

				// Bulle dynamique (mesure ET rendu avec la même police que les PV)
				if ( m_showOverheadBubbles )
				{
					auto [bubbleTxt, bubbleDigitsCol] = BuildBubbleValue( name, id );
					if ( IsCharacterDead( id ) )
						bubbleDigitsCol = IM_COL32( 160, 160, 160, 255 );

					if ( !bubbleTxt.empty() )
					{
						float w3 = MeasureTintedLineWidth( overlayFont, overlayFS, bubbleTxt );
						// Décalage vertical cohérent avec la taille réelle des PV
						float baseYOffset = ( m_showOverheadHP ? hpSize.y * 0.5f : 0.0f ) + 30.0f;
						ImVec2 pos = ImVec2( screen.x - w3 * 0.5f, screen.y + baseYOffset );

						DrawTintedNumbersML( overlayFont, overlayFS, pos, bubbleTxt,
											 IM_COL32( 205, 1, 255, 230 ), // texte non-numérique (libellés)
											 bubbleDigitsCol,			   // chiffres (teinte perso)
											 -1, false, 2.f );
					}
				}
			}
		}

		// ── Chrono central ─────────────────────────────────────────────────────
		if ( s_showMenu && s_timerVisible )
		{
			double now = ImGui::GetTime();
			double effective = s_timerPaused ? s_pauseBegin : now;
			int secs = (int)std::floor( ( effective - s_timerStart - s_totalPaused ) + 1e-9 );
			if ( secs < 0 )
				secs = 0;
			int mm = secs / 60, ss = secs % 60;

			char tbuf[16];
			snprintf( tbuf, sizeof tbuf, "%02d:%02d", mm, ss );
			ImVec2 disp = ImGui::GetIO().DisplaySize;
			ImVec2 ts = fontMax->CalcTextSizeA( fontMax->FontSize, FLT_MAX, 0.f, tbuf );
			ImVec2 p = ImVec2( std::round( disp.x * 0.5f - ts.x * 0.5f ), std::round( disp.y * 0.5f - ts.y * 0.5f ) );
			DrawTextOutlined( fontMax, fontMax->FontSize, p, IM_COL32( 255, 255, 255, 255 ), tbuf );
		}
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Helpers privés appelés par Step()
	// ─────────────────────────────────────────────────────────────────────────────
private:
	// Overhead (au-dessus des personnages)
	bool m_showOverheadHP = true;
	bool m_showOverheadBubbles = true;

	bool WorldReady() const
	{
		return B2_IS_NON_NULL( m_worldId ) && B2_IS_NON_NULL( m_groundId );
	}
	void BeginFrame( double /*now*/ )
	{
		m_hammerJustInverted = false;
		// (autres resets per-frame à ajouter ici si besoin)
	}
	void PrePhysicsUpdate( double now )
	{
		// ① ÉVÉNEMENTS du step précédent → appliquent effets/kill/gel
		HandleProjectileWallContacts();
		HandleProjectileWeaponContacts();
		ProcessHitSensors();			 // crée les hit-freezes ici
		ProcessProjectileDestructions(); // supprime/trigger explosion MAINTENANT
		ProcessPendingCharacterDeaths(); // morts avant la physique

		// ② Appliquer/maintenir les freezes AVANT la physique
		UpdateFreezes();

		// ③ Gameplay qui doit bouger DANS CE STEP
		AutoFireAll( now );	  // tirs spawnent et bougent dès ce step
		UpdateTurrets( now ); // idem
							  // (⚠️ On retire MaintainMinVelocityForAll d’ici)
	}
	void DoPhysicsStep()
	{
		Sample::Step(); // (fait b2World_Step)
	}
	void PostPhysicsUpdate( double now )
	{
		// Événements du step courant
		ResolveWeaponMotorInversions( now );
		TrackWeaponWeaponSticks( now );
		HandleCharacterReboundContacts(); // sons char↔mur/tourelle
	}
	void AnimateAndCompute( double now )
	{
		AnimateWeaponPassives();
		UpdateUnarmedSpeedRampAndDamage();
		UpdatehammerDamageProgression( now );
	}
	void ApplyStatuses( double /*now*/ )
	{
		// États continus (ne dépendent pas des contacts immédiats)
		UpdatePoison();
		UpdateSlashes();
	}
	void CleanupAndFinalize( double /*now*/ )
	{
		ProcessProjectileDestructions();
		ProcessPendingCharacterDeaths();
		UpdateFreezes();
	}

	// ========= Spawners (pointeurs de membres) =========

	const std::unordered_map<std::string, void ( WeaponsBallsVS::* )( const b2Vec2& )> spawnFuncs = {
		{ "Bow", &WeaponsBallsVS::CreateCharacterBow },
		{ "Crossbow", &WeaponsBallsVS::CreateCharacterCrossbow },
		{ "Sword", &WeaponsBallsVS::CreateCharacterSword },
		{ "Axe", &WeaponsBallsVS::CreateCharacterAxe },
		{ "Dagger", &WeaponsBallsVS::CreateCharacteDagger },
		{ "Katana", &WeaponsBallsVS::CreateCharacterKatana },
		{ "Trident", &WeaponsBallsVS::CreateCharacterTrident },
		{ "Hammer", &WeaponsBallsVS::CreateCharacterhammer },
		{ "Poison_Blowgun", &WeaponsBallsVS::CreateCharacterPoisonBlowgun },
		{ "Club", &WeaponsBallsVS::CreateCharacterClub },
		{ "Scythe", &WeaponsBallsVS::CreateCharacterScythe },
		{ "Spear", &WeaponsBallsVS::CreateCharacterSpear },
		{ "Kunai", &WeaponsBallsVS::CreateCharacterKunai },
		{ "Boomerang", &WeaponsBallsVS::CreateCharacterBoomerang },
		{ "Shuriken", &WeaponsBallsVS::CreateCharacterShuriken },
		{ "Big_Sword", &WeaponsBallsVS::CreateCharacterBigSword },
		{ "Electric_Staff", &WeaponsBallsVS::CreateCharacterElectricStaff },
		{ "Explosion_Staff", &WeaponsBallsVS::CreateCharacterExplosionStaff },
		{ "Frost_Staff", &WeaponsBallsVS::CreateCharacterFrostStaff },
		{ "Mahoraga", &WeaponsBallsVS::CreateCharacterMahoraga },
		{ "VampireKnife", &WeaponsBallsVS::CreateCharacterVampireKnife },
		{ "Shield", &WeaponsBallsVS::CreateCharacterShield },
		{ "Wrench", &WeaponsBallsVS::CreateCharacterWrench },
		{ "Unarmed", &WeaponsBallsVS::CreateCharacterUnarmed },
		{ "Glaive", &WeaponsBallsVS::CreateCharacterGlaive },
		{ "Sickle_R", &WeaponsBallsVS::CreateCharacterSickleRight },
		{ "Flask", &WeaponsBallsVS::CreateCharacterFlask },
		{ "Lance", &WeaponsBallsVS::CreateCharacterLance },

	};

	// ========= Spawners concrets =========

	void CreateCharacterBow( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.0f, 0.0f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		bd.angularDamping = 0.0f;
		m_characterIdBow = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdBow )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorYellow;

		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdBow, &skinSd, &circSkin );

		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdBow );
		m_characterSkinShape[b2StoreBodyId( m_characterIdBow )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorYellow;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;

		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdBow, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "BOW" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdBow );
		w.isBullet = false;
		m_bowId = b2CreateBody( m_worldId, &w );

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );

		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( t == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_bowId, &sd, &box );
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdBow ) );
					b2CreatePolygonShape( m_bowId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdBow;
		jd.base.bodyIdB = m_bowId;
		jd.base.localFrameB.p = { -1.0f, -1.0f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_bowJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_bowId )] = m_bowJointId;
		m_weaponOwner[b2StoreBodyId( m_bowId )] = m_characterIdBow;
		m_characterWeapon[b2StoreBodyId( m_characterIdBow )] = m_bowId;
	}

	void CreateCharacterCrossbow( const b2Vec2& pos )
	{

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.0f, 0.0f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdCrossbowBow = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdCrossbowBow )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorDarkGoldenRod;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdCrossbowBow, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdCrossbowBow );
		m_characterSkinShape[b2StoreBodyId( m_characterIdCrossbowBow )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorDarkGoldenRod;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdCrossbowBow, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "CROSSBOW" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdCrossbowBow );
		w.isBullet = false;
		m_crossbowId = b2CreateBody( m_worldId, &w );

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );

		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( t == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_crossbowId, &sd, &box );
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdCrossbowBow ) );
					b2CreatePolygonShape( m_crossbowId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdCrossbowBow;
		jd.base.bodyIdB = m_crossbowId;
		jd.base.localFrameB.p = { -2.0f, -2.0f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_crossbowJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_crossbowId )] = m_crossbowJointId;
		m_weaponOwner[b2StoreBodyId( m_crossbowId )] = m_characterIdCrossbowBow;
		m_characterWeapon[b2StoreBodyId( m_characterIdCrossbowBow )] = m_crossbowId;
	}

	void CreateCharacterVampireKnife( const b2Vec2& pos )
	{

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 4.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdVampireKnife = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdVampireKnife )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorCrimson;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdVampireKnife, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdVampireKnife );
		m_characterSkinShape[b2StoreBodyId( m_characterIdVampireKnife )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorCrimson;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdVampireKnife, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "VAMPIRE_KNIFE" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdVampireKnife );
		w.isBullet = false;
		m_vampireKnifeId = b2CreateBody( m_worldId, &w );

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;

		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_vampireKnifeId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdVampireKnife );
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.enableContactEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;

					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_vampireKnifeId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdVampireKnife;
		jd.base.bodyIdB = m_vampireKnifeId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_vampireKnifeJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_vampireKnifeId )] = m_vampireKnifeJointId;
		m_weaponOwner[b2StoreBodyId( m_vampireKnifeId )] = m_characterIdVampireKnife;
		m_characterWeapon[b2StoreBodyId( m_characterIdVampireKnife )] = m_vampireKnifeId;
	}

	void CreateCharacterSword( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdSword = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdSword )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorIndianRed;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdSword, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdSword );
		m_characterSkinShape[b2StoreBodyId( m_characterIdSword )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef bodySd = b2DefaultShapeDef();
		bodySd.density = 20.0f;
		bodySd.material = b2DefaultSurfaceMaterial();
		bodySd.material.customColor = b2_colorIndianRed;
		bodySd.material.restitution = 1.0f;
		bodySd.filter.categoryBits = CATEGORY_CHARACTER;
		bodySd.filter.maskBits = 0xFFFF;
		bodySd.enableHitEvents = true;
		b2Circle circBody = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdSword, &bodySd, &circBody );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "SWORD" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdSword );
		w.isBullet = false;
		m_swordId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_swordId, true );
		m_weaponDamage[b2StoreBodyId( m_swordId )] = 1;

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.enableContactEvents = true;
				float lx = ( float( x ) + 0.5f - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( float( y ) + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				sd.isSensor = true;
				if ( t == Pixel_Sensor )
				{
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.enableContactEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
				}
				sd.material.customColor = art->at( x, y );
				b2CreatePolygonShape( m_swordId, &sd, &box );
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdSword;
		jd.base.bodyIdB = m_swordId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_swordJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_swordId )] = m_swordJointId;
		m_weaponOwner[b2StoreBodyId( m_swordId )] = m_characterIdSword;
		m_characterWeapon[b2StoreBodyId( m_characterIdSword )] = m_swordId;
	}

	void CreateCharacteDagger( const b2Vec2& pos )
	{
		b2BodyDef d = b2DefaultBodyDef();
		d.type = b2_dynamicBody;
		d.position = pos;
		d.linearVelocity = { 5.f, 0.f };
		d.motionLocks = { false, false, true };
		d.linearDamping = 0.0f;
		m_characterIdDagger = b2CreateBody( m_worldId, &d );
		m_characterHP[b2StoreBodyId( m_characterIdDagger )] = 100;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorLime;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdDagger, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorLime;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdDagger, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdDagger );
		m_characterSkinShape[b2StoreBodyId( m_characterIdDagger )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "DAGGER" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdDagger );
		w.isBullet = false;
		m_daggerId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_daggerId, true );
		m_weaponDamage[b2StoreBodyId( m_daggerId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = 0.18;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_daggerId, &sd, &box );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_daggerId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdDagger;
		jd.base.bodyIdB = m_daggerId;
		jd.base.localFrameB.p = { -2.2f, -2.2f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0;
		m_daggerJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_daggerId )] = m_daggerJointId;
		m_weaponOwner[b2StoreBodyId( m_daggerId )] = m_characterIdDagger;
		m_characterWeapon[b2StoreBodyId( m_characterIdDagger )] = m_daggerId;
	}

	void CreateCharacterAxe( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.0f, 0.0f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdAxe = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdAxe )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorDarkSlateGray;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdAxe, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdAxe );
		m_characterSkinShape[b2StoreBodyId( m_characterIdAxe )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef bodySd = b2DefaultShapeDef();
		bodySd.density = 20.0f;
		bodySd.material = b2DefaultSurfaceMaterial();
		bodySd.material.customColor = b2_colorDarkSlateGray;
		bodySd.material.restitution = 1.0f;
		bodySd.filter.categoryBits = CATEGORY_CHARACTER;
		bodySd.filter.maskBits = 0xFFFF;
		bodySd.enableHitEvents = true;
		b2Circle circBody = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdAxe, &bodySd, &circBody );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "AXE" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdAxe );
		w.isBullet = false;
		m_axeId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_axeId, true );

		m_weaponDamage[b2StoreBodyId( m_axeId )] = 1;

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );
		float px = kPixelSize;

		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.enableContactEvents = true;

				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( t == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_axeId, &sd, &box );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_axeId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdAxe;
		jd.base.bodyIdB = m_axeId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_axeJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_axeId )] = m_axeJointId;
		m_weaponOwner[b2StoreBodyId( m_axeId )] = m_characterIdAxe;
		m_characterWeapon[b2StoreBodyId( m_characterIdAxe )] = m_axeId;
	}

	void CreateCharacterKatana( const b2Vec2& pos )
	{
		b2BodyDef d = b2DefaultBodyDef();
		d.type = b2_dynamicBody;
		d.position = pos;
		d.linearVelocity = { 5.f, 0.f };
		d.motionLocks = { false, false, true };
		d.linearDamping = 0.0f;
		m_characterIdKatana = b2CreateBody( m_worldId, &d );
		m_characterHP[b2StoreBodyId( m_characterIdKatana )] = 100;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorAquamarine;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdKatana, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorAquamarine;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdKatana, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdKatana );
		m_characterSkinShape[b2StoreBodyId( m_characterIdKatana )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "KATANA" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdKatana );
		w.isBullet = false;
		m_katanaId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_katanaId, true );
		m_weaponDamage[b2StoreBodyId( m_katanaId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;

		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_katanaId, &sd, &box );
				}
				else
				{
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.isSensor = false;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_katanaId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdKatana;
		jd.base.bodyIdB = m_katanaId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_katanaJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_katanaId )] = m_katanaJointId;
		m_weaponOwner[b2StoreBodyId( m_katanaId )] = m_characterIdKatana;
		m_characterWeapon[b2StoreBodyId( m_characterIdKatana )] = m_katanaId;
	}

	void CreateCharacterTrident( const b2Vec2& pos )
	{
		b2BodyDef d = b2DefaultBodyDef();
		d.type = b2_dynamicBody;
		d.position = pos;
		d.linearVelocity = { 5.f, 0.f };
		d.motionLocks = { false, false, true };
		d.linearDamping = 0.0f;
		m_characterIdTrident = b2CreateBody( m_worldId, &d );
		m_characterHP[b2StoreBodyId( m_characterIdTrident )] = 100;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorNavy;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdTrident, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorNavy;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdTrident, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdTrident );
		m_characterSkinShape[b2StoreBodyId( m_characterIdTrident )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "TRIDENT" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdTrident );
		w.isBullet = false;
		m_tridentId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_tridentId, true );
		m_weaponDamage[b2StoreBodyId( m_tridentId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;

		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_tridentId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdTrident );
				}
				else
				{
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.isSensor = false;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_tridentId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdTrident;
		jd.base.bodyIdB = m_tridentId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_tridentJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_tridentId )] = m_tridentJointId;
		m_weaponOwner[b2StoreBodyId( m_tridentId )] = m_characterIdTrident;
		m_characterWeapon[b2StoreBodyId( m_characterIdTrident )] = m_tridentId;
	}

	void CreateCharacterSpear( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdSpear = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdSpear )] = 100;
		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material = b2DefaultSurfaceMaterial();
		skinSd.material.customColor = b2_colorCyan;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdSpear, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdSpear );
		m_characterSkinShape[b2StoreBodyId( m_characterIdSpear )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;
		b2ShapeDef bodySd = b2DefaultShapeDef();
		bodySd.density = 20.0f;
		bodySd.material = b2DefaultSurfaceMaterial();
		bodySd.material.customColor = b2_colorCyan;
		bodySd.material.restitution = 1.0f;
		bodySd.filter.categoryBits = CATEGORY_CHARACTER;
		bodySd.filter.maskBits = 0xFFFF;
		bodySd.enableHitEvents = true;
		b2Circle circBody = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdSpear, &bodySd, &circBody );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;
		const PixelArtColor* art = PixelArtColor_GetByName( "SPEAR_LVL1" );
		if ( !art )
			return;

		m_spearLevel = 1;
		m_spearPixelSize = kPixelSize;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdSpear );
		w.isBullet = false;
		m_spearId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_spearId, true );
		m_weaponDamage[b2StoreBodyId( m_spearId )] = 1;
		BuildSpearFromSprite( art, m_spearPixelSize );
		m_weaponOwner[b2StoreBodyId( m_spearId )] = m_characterIdSpear;
		m_characterWeapon[b2StoreBodyId( m_characterIdSpear )] = m_spearId;
	}

	void CreateCharacterMahoraga( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdMahoraga = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdMahoraga )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorGold;
		b2Circle circSkin = { { 0, 0 }, 1.f };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdMahoraga, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdMahoraga );
		m_characterSkinShape[b2StoreBodyId( m_characterIdMahoraga )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.material = b2DefaultSurfaceMaterial();
		cs.density = 20.0f;
		cs.material.customColor = b2_colorGold;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, 1.f };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdMahoraga, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "MAHORAGA" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdMahoraga );
		w.isBullet = false;
		m_mahoragaId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_mahoragaId, true );
		m_weaponDamage[b2StoreBodyId( m_mahoragaId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_mahoragaId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdMahoraga );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_mahoragaId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdMahoraga;
		jd.base.bodyIdB = m_mahoragaId;
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_mahoragaJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_mahoragaId )] = m_mahoragaJointId;
		m_weaponOwner[b2StoreBodyId( m_mahoragaId )] = m_characterIdMahoraga;
		m_characterWeapon[b2StoreBodyId( m_characterIdMahoraga )] = m_mahoragaId;
	}

	void CreateCharacterScythe( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdScythe = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdScythe )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorBlueViolet;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdScythe, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdScythe );
		m_characterSkinShape[b2StoreBodyId( m_characterIdScythe )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorBlueViolet;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdScythe, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "SCYTHE" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdScythe );
		w.isBullet = false;
		m_scytheId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_scytheId, true );
		m_weaponDamage[b2StoreBodyId( m_scytheId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_scytheId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdScythe );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_scytheId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdScythe;
		jd.base.bodyIdB = m_scytheId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_scytheJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_scytheId )] = m_scytheJointId;
		m_weaponOwner[b2StoreBodyId( m_scytheId )] = m_characterIdScythe;
		m_characterWeapon[b2StoreBodyId( m_characterIdScythe )] = m_scytheId;
	}

	void CreateCharacterBigSword( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdBigSword = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdBigSword )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorDarkOrange;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdBigSword, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdBigSword );
		m_characterSkinShape[b2StoreBodyId( m_characterIdBigSword )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorDarkOrange;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdBigSword, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "BIG_SWORD" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdBigSword );
		w.isBullet = false;
		m_bigSwordId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_bigSwordId, true );
		m_weaponDamage[b2StoreBodyId( m_bigSwordId )] = 2;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_bigSwordId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdBigSword );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_bigSwordId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdBigSword;
		jd.base.bodyIdB = m_bigSwordId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_bigSwordJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_bigSwordId )] = m_bigSwordJointId;
		m_weaponOwner[b2StoreBodyId( m_bigSwordId )] = m_characterIdBigSword;
		m_characterWeapon[b2StoreBodyId( m_characterIdBigSword )] = m_bigSwordId;
	}

	void CreateCharacterhammer( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 4.5f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdhammer = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdhammer )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorMediumVioletRed;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdhammer, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdhammer );
		m_characterSkinShape[b2StoreBodyId( m_characterIdhammer )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorMediumVioletRed;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdhammer, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "HAMMER" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdhammer );
		w.isBullet = false;
		m_hammerId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_hammerId, true );
		m_weaponDamage[b2StoreBodyId( m_hammerId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_hammerId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdhammer );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_hammerId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdhammer;
		jd.base.bodyIdB = m_hammerId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_hammerJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_hammerId )] = m_hammerJointId;
		m_weaponOwner[b2StoreBodyId( m_hammerId )] = m_characterIdhammer;
		m_characterWeapon[b2StoreBodyId( m_characterIdhammer )] = m_hammerId;
	}

	void CreateCharacterPoisonBlowgun( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdPoisonBlowgun = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdPoisonBlowgun )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorGreenYellow;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdPoisonBlowgun, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdPoisonBlowgun );
		m_characterSkinShape[b2StoreBodyId( m_characterIdPoisonBlowgun )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorGreenYellow;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdPoisonBlowgun, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "POISON_BLOWGUN" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdPoisonBlowgun );
		w.isBullet = false;
		m_poisonBlowgunId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_poisonBlowgunId, true );
		m_weaponDamage[b2StoreBodyId( m_poisonBlowgunId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_poisonBlowgunId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdPoisonBlowgun );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.material.customColor = art->at( x, y );
					// ARME (pixels physiques)
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.filter.groupIndex = MakeTurretGroup( m_characterIdPoisonBlowgun ); // ✔️

					b2CreatePolygonShape( m_poisonBlowgunId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdPoisonBlowgun;
		jd.base.bodyIdB = m_poisonBlowgunId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_poisonBlowgunJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_poisonBlowgunId )] = m_poisonBlowgunJointId;
		m_weaponOwner[b2StoreBodyId( m_poisonBlowgunId )] = m_characterIdPoisonBlowgun;
		m_characterWeapon[b2StoreBodyId( m_characterIdPoisonBlowgun )] = m_poisonBlowgunId;
	}

	void CreateCharacterClub( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, false };
		bd.linearDamping = 0.0f;
		m_characterIdClub = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdClub )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorSaddleBrown;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdClub, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdClub );
		m_characterSkinShape[b2StoreBodyId( m_characterIdClub )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorSaddleBrown;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdClub, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "CLUB" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdClub );
		w.isBullet = false;
		m_clubId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_clubId, true );
		m_weaponDamage[b2StoreBodyId( m_clubId )] = 2;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_clubId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdClub );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_CHARACTER;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_clubId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdClub;
		jd.base.bodyIdB = m_clubId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_clubJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_clubId )] = m_clubJointId;
		m_weaponOwner[b2StoreBodyId( m_clubId )] = m_characterIdClub;
		m_characterWeapon[b2StoreBodyId( m_characterIdClub )] = m_clubId;
	}

	void CreateCharacterKunai( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdKunai = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdKunai )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorIndigo;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdKunai, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdKunai );
		m_characterSkinShape[b2StoreBodyId( m_characterIdKunai )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorIndigo;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdKunai, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "KUNAI" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdKunai );
		w.isBullet = false;
		m_kunaiId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_kunaiId, true );
		m_weaponDamage[b2StoreBodyId( m_kunaiId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_kunaiId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdKunai );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_kunaiId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdKunai;
		jd.base.bodyIdB = m_kunaiId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_kunaiJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_kunaiId )] = m_kunaiJointId;
		m_weaponOwner[b2StoreBodyId( m_kunaiId )] = m_characterIdKunai;
		m_characterWeapon[b2StoreBodyId( m_characterIdKunai )] = m_kunaiId;
	}

	void CreateCharacterBoomerang( const b2Vec2& pos )
	{
		// ---- perso ----
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdBoomerang = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdBoomerang )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorLightGreen;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdBoomerang, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdBoomerang );
		m_characterSkinShape[b2StoreBodyId( m_characterIdBoomerang )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorLightGreen;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdBoomerang, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		// ---- sprite ----
		const PixelArtColor* art = PixelArtColor_GetByName( "BOOMERANG" );
		if ( !art )
			return;

		// ---- boomerang (arme) ----
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdBoomerang );
		w.isBullet = false;
		m_boomerangId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_boomerangId, true );
		m_weaponDamage[b2StoreBodyId( m_boomerangId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;

				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_boomerangId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdBoomerang );
					m_shapeBaseColor[b2StoreShapeId( shapeId )] = sd.material.customColor; // (utile pour les tints éventuels)
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId s = b2CreatePolygonShape( m_boomerangId, &sd, &box );
					m_shapeBaseColor[b2StoreShapeId( s )] = sd.material.customColor;
				}
			}

		// ---- joint revolute ----
		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdBoomerang;
		jd.base.bodyIdB = m_boomerangId;

		// ancrage au centre (boomerang animé autour de 0,0 comme convenu)
		jd.base.localFrameB.p = { 0.0f, 0.0f };

		// moteur “comme les autres”
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign ); // ← même valeur que shuriken
		jd.maxMotorTorque = 50.0f;

		m_boomerangJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		// ---- mappings génériques ----
		m_weaponToJoint[b2StoreBodyId( m_boomerangId )] = m_boomerangJointId;
		m_weaponOwner[b2StoreBodyId( m_boomerangId )] = m_characterIdBoomerang;
		m_characterWeapon[b2StoreBodyId( m_characterIdBoomerang )] = m_boomerangId;

		// ---- ✅ enregistrement état “multi-boomerangs” ----
		{
			const uint64_t wid = b2StoreBodyId( m_boomerangId );
			BoomerangState s;
			s.body = m_boomerangId;
			s.phase = 0.0;
			s.lastTime = ImGui::GetTime();
			s.reachA = m_boomerangReachA; // défauts init (peuvent venir du GUI)
			s.reachB = m_boomerangReachB;
			s.animSpeed = m_boomerangAnimSpeed;
			s.hitCount = 0;
			m_boomerangs[wid] = s;
		}
	}

	void CreateCharacterShuriken( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdShuriken = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdShuriken )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorOliveDrab;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdShuriken, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdShuriken );
		m_characterSkinShape[b2StoreBodyId( m_characterIdShuriken )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorOliveDrab;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdShuriken, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "SHURIKEN" );
		if ( !art )
			return;
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdShuriken );
		w.isBullet = false;
		m_shurikenId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_shurikenId, true );
		m_weaponDamage[b2StoreBodyId( m_shurikenId )] = 1;
		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_shurikenId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdShuriken );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdShuriken ) );

					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_shurikenId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdShuriken;
		jd.base.bodyIdB = m_shurikenId;
		jd.base.localFrameB.p = { -1.5f, -1.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_shurikenJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_shurikenId )] = m_shurikenJointId;
		m_weaponOwner[b2StoreBodyId( m_shurikenId )] = m_characterIdShuriken;
		m_characterWeapon[b2StoreBodyId( m_characterIdShuriken )] = m_shurikenId;
	}

	void CreateCharacterElectricStaff( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdElectricStaff = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdElectricStaff )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorBox2DYellow;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdElectricStaff, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdElectricStaff );
		m_characterSkinShape[b2StoreBodyId( m_characterIdElectricStaff )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorBox2DYellow;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdElectricStaff, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "ELECTRIC_STAFF" );
		if ( !art )
			return;
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdElectricStaff );
		w.isBullet = false;
		m_electricStaffId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_electricStaffId, true );
		m_weaponDamage[b2StoreBodyId( m_electricStaffId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_electricStaffId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdElectricStaff );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdElectricStaff ) );

					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_electricStaffId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdElectricStaff;
		jd.base.bodyIdB = m_electricStaffId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_electricStaffJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_electricStaffId )] = m_electricStaffJointId;
		m_weaponOwner[b2StoreBodyId( m_electricStaffId )] = m_characterIdElectricStaff;
		m_characterWeapon[b2StoreBodyId( m_characterIdElectricStaff )] = m_electricStaffId;
	}

	void CreateCharacterExplosionStaff( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdExplosionStaff = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdExplosionStaff )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorRed;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdExplosionStaff, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdExplosionStaff );
		m_characterSkinShape[b2StoreBodyId( m_characterIdExplosionStaff )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorRed;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdExplosionStaff, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "EXPLOSION_STAFF" );
		if ( !art )
			return;
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdExplosionStaff );
		w.isBullet = false;
		m_explosionStaffId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_explosionStaffId, true );
		m_weaponDamage[b2StoreBodyId( m_explosionStaffId )] = 0;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_explosionStaffId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdExplosionStaff );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdExplosionStaff ) );

					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_explosionStaffId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdExplosionStaff;
		jd.base.bodyIdB = m_explosionStaffId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_explosionStaffJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_explosionStaffId )] = m_explosionStaffJointId;
		m_weaponOwner[b2StoreBodyId( m_explosionStaffId )] = m_characterIdExplosionStaff;
		m_characterWeapon[b2StoreBodyId( m_characterIdExplosionStaff )] = m_explosionStaffId;
	}

	void CreateCharacterFrostStaff( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdFrostStaff = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdFrostStaff )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorAquamarine;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdFrostStaff, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdFrostStaff );
		m_characterSkinShape[b2StoreBodyId( m_characterIdFrostStaff )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorAquamarine;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdFrostStaff, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "FROST_STAFF" );
		if ( !art )
			return;
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdFrostStaff );
		w.isBullet = false;
		m_frostStaffId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_frostStaffId, true );
		m_weaponDamage[b2StoreBodyId( m_frostStaffId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;
				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_frostStaffId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdFrostStaff );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_frostStaffId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdFrostStaff;
		jd.base.bodyIdB = m_frostStaffId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_frostStaffJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_frostStaffId )] = m_frostStaffJointId;
		m_weaponOwner[b2StoreBodyId( m_frostStaffId )] = m_characterIdFrostStaff;
		m_characterWeapon[b2StoreBodyId( m_characterIdFrostStaff )] = m_frostStaffId;
	}

	void CreateCharacterShield( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 0.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdShield = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdShield )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorDarkGoldenRod;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdShield, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdShield );
		m_characterSkinShape[b2StoreBodyId( m_characterIdShield )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.restitution = 1.0f;
		cs.material.customColor = b2_colorDarkGoldenRod;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyId = b2CreateCircleShape( m_characterIdShield, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "SHIELD" );
		if ( !art )
			return;

		// --- Bouclier (arme) orienté +45° (haut-droite)
		const float kFaceUpRight = b2_pi * 0.25f; // 45°
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdShield );
		w.rotation = b2MakeRot( kFaceUpRight ); // ✅ sprite/shape tourné
		w.isBullet = false;
		m_shieldId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_shieldId, true );
		m_weaponDamage[b2StoreBodyId( m_shieldId )] = 1;

		std::vector<PixelPhysicsType> physMap;
		ComputePixelPhysicsMap( *art, physMap );

		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType type = physMap[y * art->width + x];
				if ( type == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;

				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( type == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_shieldId, &sd, &box );
					m_shapeToCharacter[b2StoreShapeId( shapeId )] = b2StoreBodyId( m_characterIdShield );
				}
				else
				{
					sd.isSensor = false;
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_shieldId, &sd, &box );
				}
			}

		// --- Revolute joint avec ancre corrigée (en local B)
		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdShield;
		jd.base.bodyIdB = m_shieldId;

		// Direction voulue en MONDE (l’ancienne diagonale). Change le signe si nécessaire.
		const b2Vec2 desiredWorldAnchor = { -2.0f, -2.0f };

		// local = R(-θ) * world   (θ = +45°)
		const float c = std::cos( kFaceUpRight );
		const float s = std::sin( kFaceUpRight );
		const b2Vec2 localB = { c * desiredWorldAnchor.x + s * desiredWorldAnchor.y,
								-s * desiredWorldAnchor.x + c * desiredWorldAnchor.y };
		jd.base.localFrameB.p = localB; // ≈ {-2.8284f, 0.0f} pour {-2,-2}

		jd.enableMotor = false;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_shieldJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_shieldId )] = m_shieldJointId;
		m_weaponOwner[b2StoreBodyId( m_shieldId )] = m_characterIdShield;
		m_characterWeapon[b2StoreBodyId( m_characterIdShield )] = m_shieldId;
	}

	void CreateCharacterWrench( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.0f, 0.0f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdWrench = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdWrench )] = 100;

		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.enableContactEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorOrange;
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdWrench, &skinSd, &circSkin );

		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdWrench );
		m_characterSkinShape[b2StoreBodyId( m_characterIdWrench )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorOrange;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;

		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdWrench, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = cs.material.customColor;

		const PixelArtColor* art = PixelArtColor_GetByName( "WRENCH" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdWrench );
		w.isBullet = false;
		m_wrenchId = b2CreateBody( m_worldId, &w );

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );

		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				if ( t == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2ShapeId shapeId = b2CreatePolygonShape( m_wrenchId, &sd, &box );
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.enableContactEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_wrenchId, &sd, &box );
				}
			}

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdWrench;
		jd.base.bodyIdB = m_wrenchId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_wrenchJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_wrenchId )] = m_wrenchJointId;
		m_weaponOwner[b2StoreBodyId( m_wrenchId )] = m_characterIdWrench;
		m_characterWeapon[b2StoreBodyId( m_characterIdWrench )] = m_wrenchId;
		m_weaponDamage[b2StoreBodyId( m_wrenchId )] = 1;
	}

	void CreateCharacterUnarmed( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 6.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		bd.isBullet = false;
		m_characterIdUnarmed = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdUnarmed )] = 100;

		b2ShapeDef sk = b2DefaultShapeDef();
		sk.isSensor = true;
		sk.enableSensorEvents = true;
		sk.filter.categoryBits = CATEGORY_SKIN;
		sk.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		sk.material = b2DefaultSurfaceMaterial();
		sk.material.customColor = b2_colorLightGray;
		b2Circle skin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinId = b2CreateCircleShape( m_characterIdUnarmed, &sk, &skin );
		m_shapeToCharacter[b2StoreShapeId( skinId )] = b2StoreBodyId( m_characterIdUnarmed );
		m_characterSkinShape[b2StoreBodyId( m_characterIdUnarmed )] = skinId;
		m_shapeBaseColor[b2StoreShapeId( skinId )] = sk.material.customColor;

		b2ShapeDef bodySd = b2DefaultShapeDef();
		bodySd.density = 0.0001f;
		bodySd.material = b2DefaultSurfaceMaterial();
		bodySd.material.restitution = 1.0f;
		bodySd.material.customColor = b2_colorLightGray;
		bodySd.filter.categoryBits = CATEGORY_CHARACTER;
		bodySd.filter.maskBits = 0xFFFF;
		b2Circle phys = { { 0, 0 }, kCharacterRadius };
		b2ShapeId physId = b2CreateCircleShape( m_characterIdUnarmed, &bodySd, &phys );
		m_shapeBaseColor[b2StoreShapeId( physId )] = bodySd.material.customColor;

		b2ShapeDef ringSd = b2DefaultShapeDef();
		ringSd.isSensor = true;
		ringSd.enableSensorEvents = true;
		ringSd.filter.categoryBits = CATEGORY_WEAPON;
		ringSd.filter.maskBits = CATEGORY_SKIN;
		ringSd.material = b2DefaultSurfaceMaterial();
		ringSd.material.customColor = b2_colorLightGray;

		b2Circle ring = { { 0, 0 }, kCharacterRadius + kUnarmedRingPad };
		b2ShapeId ringId = b2CreateCircleShape( m_characterIdUnarmed, &ringSd, &ring );

		m_unarmedGhostRadius = ring.radius;

		const uint64_t wid = b2StoreBodyId( m_characterIdUnarmed );
		m_weaponDamage[wid] = 1;
		m_weaponOwner[wid] = m_characterIdUnarmed; // ← owner bien renseigné
		m_characterWeapon[b2StoreBodyId( m_characterIdUnarmed )] = m_characterIdUnarmed;
	}

	void SummonTurretFromWrench( const b2Vec2& pos )
	{
		b2BodyDef anchorDef = b2DefaultBodyDef();
		anchorDef.type = b2_staticBody;
		anchorDef.position = pos;
		b2BodyId anchorId = b2CreateBody( m_worldId, &anchorDef );

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.gravityScale = 0.0f;
		bd.isBullet = false;
		b2BodyId turretId = b2CreateBody( m_worldId, &bd );

		int16_t turretGroup = MakeTurretGroup( turretId );

		b2ShapeDef circDef = b2DefaultShapeDef();
		circDef.density = 100.0f;
		circDef.enableContactEvents = true;
		circDef.filter.categoryBits = CATEGORY_TURRET;
		circDef.filter.maskBits = 0xFFFF & ~CATEGORY_TURRET;
		circDef.filter.groupIndex = turretGroup;
		circDef.material = b2DefaultSurfaceMaterial();
		circDef.material.customColor = b2_colorOrange;
		circDef.material.restitution = 1.0f;

		// +20%
		b2Circle circ = { { 0, 0 }, 0.66f }; // 0.55f * 1.2
		b2CreateCircleShape( turretId, &circDef, &circ );

		b2ShapeDef boxDef = circDef;
		boxDef.enableContactEvents = true;

		// +20% (hx, hy, offset-x)
		b2Polygon box = b2MakeOffsetBox( 0.30f, 0.18f, { 0.96f, 0.0f }, b2MakeRot( 0 ) );
		// (0.25f, 0.15f, {0.55f + 0.25f, 0}) -> (0.30f, 0.18f, {0.66f + 0.30f = 0.96f, 0})

		b2CreatePolygonShape( turretId, &boxDef, &box );

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = anchorId;
		jd.base.bodyIdB = turretId;
		jd.enableMotor = true;
		jd.motorSpeed = 1.5f;
		jd.maxMotorTorque = 50.0f;
		b2JointId turretJoint = b2CreateRevoluteJoint( m_worldId, &jd );

		m_turretIds.insert( turretId );
		m_turretOwner[turretId] = m_characterIdWrench;
	}

	void CreateCharacterGlaive( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdGlaive = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdGlaive )] = 100;

		// Skin (sensor)
		{
			b2ShapeDef skinSd = b2DefaultShapeDef();
			skinSd.isSensor = true;
			skinSd.enableSensorEvents = true;
			skinSd.filter.categoryBits = CATEGORY_SKIN;
			skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
			skinSd.material.customColor = b2_colorWheat;

			b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
			b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdGlaive, &skinSd, &circSkin );
			m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdGlaive );
			m_characterSkinShape[b2StoreBodyId( m_characterIdGlaive )] = skinShapeId;
			m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;
		}

		// Corps solide
		{
			b2ShapeDef bodySd = b2DefaultShapeDef();
			bodySd.density = 20.0f;
			bodySd.material = b2DefaultSurfaceMaterial();
			bodySd.material.customColor = b2_colorWheat;
			bodySd.material.restitution = 1.0f;
			bodySd.filter.categoryBits = CATEGORY_CHARACTER;
			bodySd.filter.maskBits = 0xFFFF;
			bodySd.enableHitEvents = true;

			b2Circle circBody = { { 0, 0 }, kCharacterRadius };
			b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdGlaive, &bodySd, &circBody );
			m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;
		}

		const PixelArtColor* art = PixelArtColor_GetByName( "GLAIVE" );
		if ( !art )
			return;

		// Arme
		{
			b2BodyDef w = b2DefaultBodyDef();
			w.type = b2_dynamicBody;
			w.position = b2Body_GetPosition( m_characterIdGlaive );
			w.isBullet = false;
			m_glaiveId = b2CreateBody( m_worldId, &w );
			b2Body_EnableContactEvents( m_glaiveId, true );
			m_weaponDamage[b2StoreBodyId( m_glaiveId )] = 1;

			std::vector<PixelPhysicsType> map;
			ComputePixelPhysicsMap( *art, map );
			float px = kPixelSize;

			for ( int y = 0; y < art->height; ++y )
				for ( int x = 0; x < art->width; ++x )
				{
					PixelPhysicsType t = map[y * art->width + x];
					if ( t == Pixel_Void )
						continue;

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.density = 0.001f;
					sd.material = b2DefaultSurfaceMaterial();
					sd.material.restitution = 1.0f;
					sd.enableContactEvents = true;

					float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
					float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
					b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

					if ( t == Pixel_Sensor )
					{
						sd.isSensor = true;
						sd.filter.categoryBits = CATEGORY_HITBOX;
						sd.filter.maskBits = CATEGORY_SKIN;
					}
					else
					{
						sd.isSensor = false;
						sd.enableSensorEvents = true;
						sd.enableContactEvents = true;
						sd.filter.categoryBits = CATEGORY_WEAPON;
						sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
						sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdGlaive ) );
					}
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_glaiveId, &sd, &box );
				}
		}

		// Joint
		{
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_characterIdGlaive;
			jd.base.bodyIdB = m_glaiveId;
			jd.base.localFrameB.p = { -2.5f, -2.5f };
			jd.enableMotor = true;
			jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
			jd.maxMotorTorque = 50.0f;
			m_glaiveJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		m_weaponToJoint[b2StoreBodyId( m_glaiveId )] = m_glaiveJointId;
		m_weaponOwner[b2StoreBodyId( m_glaiveId )] = m_characterIdGlaive;
		m_characterWeapon[b2StoreBodyId( m_characterIdGlaive )] = m_glaiveId;
	}

	void CreateCharacterSickleRight( const b2Vec2& pos )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdSickleR = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdSickleR )] = 100;

		// Skin
		{
			b2ShapeDef skinSd = b2DefaultShapeDef();
			skinSd.isSensor = true;
			skinSd.enableSensorEvents = true;
			skinSd.filter.categoryBits = CATEGORY_SKIN;
			skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
			skinSd.material.customColor = b2_colorGhostWhite;

			b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
			b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdSickleR, &skinSd, &circSkin );
			m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdSickleR );
			m_characterSkinShape[b2StoreBodyId( m_characterIdSickleR )] = skinShapeId;
			m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;
		}

		// Corps
		{
			b2ShapeDef bodySd = b2DefaultShapeDef();
			bodySd.density = 20.0f;
			bodySd.material = b2DefaultSurfaceMaterial();
			bodySd.material.customColor = b2_colorGhostWhite;
			bodySd.material.restitution = 1.0f;
			bodySd.filter.categoryBits = CATEGORY_CHARACTER;
			bodySd.filter.maskBits = 0xFFFF;
			bodySd.enableHitEvents = true;

			b2Circle circBody = { { 0, 0 }, kCharacterRadius };
			b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdSickleR, &bodySd, &circBody );
			m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;
		}

		const PixelArtColor* art = PixelArtColor_GetByName( "SICKLE_R" );
		if ( !art )
			return;

		// Arme
		{
			b2BodyDef w = b2DefaultBodyDef();
			w.type = b2_dynamicBody;
			w.position = b2Body_GetPosition( m_characterIdSickleR );
			w.isBullet = false;
			m_sickleRId = b2CreateBody( m_worldId, &w );
			b2Body_EnableContactEvents( m_sickleRId, true );
			m_weaponDamage[b2StoreBodyId( m_sickleRId )] = 1;

			std::vector<PixelPhysicsType> map;
			ComputePixelPhysicsMap( *art, map );
			float px = kPixelSize;

			for ( int y = 0; y < art->height; ++y )
				for ( int x = 0; x < art->width; ++x )
				{
					PixelPhysicsType t = map[y * art->width + x];
					if ( t == Pixel_Void )
						continue;

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.density = 0.001f;
					sd.material = b2DefaultSurfaceMaterial();
					sd.material.restitution = 1.0f;
					sd.enableContactEvents = true;

					float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
					float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
					b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

					if ( t == Pixel_Sensor )
					{
						sd.isSensor = true;
						sd.filter.categoryBits = CATEGORY_HITBOX;
						sd.filter.maskBits = CATEGORY_SKIN;
					}
					else
					{
						sd.isSensor = false;
						sd.enableSensorEvents = true;
						sd.enableContactEvents = true;
						sd.filter.categoryBits = CATEGORY_WEAPON;
						sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
						sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdSickleR ) );
					}
					sd.material.customColor = art->at( x, y );
					b2CreatePolygonShape( m_sickleRId, &sd, &box );
				}
		}

		// Joint
		{
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_characterIdSickleR;
			jd.base.bodyIdB = m_sickleRId;
			jd.base.localFrameB.p = { -2.5f, -2.5f };
			jd.enableMotor = true;
			jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
			jd.maxMotorTorque = 50.0f;
			m_sickleRJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		m_weaponToJoint[b2StoreBodyId( m_sickleRId )] = m_sickleRJointId;
		m_weaponOwner[b2StoreBodyId( m_sickleRId )] = m_characterIdSickleR;
		m_characterWeapon[b2StoreBodyId( m_characterIdSickleR )] = m_sickleRId;
	}

	void CreateCharacterFlask( const b2Vec2& pos )
	{
		// Corps du perso
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true };
		bd.linearDamping = 0.0f;
		m_characterIdFlask = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdFlask )] = 100;

		// Skin sensor
		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorWhite; // UI "Flask" = blanc
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdFlask, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdFlask );
		m_characterSkinShape[b2StoreBodyId( m_characterIdFlask )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		// Collider du perso
		b2ShapeDef cs = b2DefaultShapeDef();
		cs.density = 20.0f;
		cs.material = b2DefaultSurfaceMaterial();
		cs.material.customColor = b2_colorWhite;
		cs.material.restitution = 1.0f;
		cs.filter.categoryBits = CATEGORY_CHARACTER;
		cs.filter.maskBits = 0xFFFF;
		cs.enableHitEvents = true;
		b2Circle circ = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdFlask, &cs, &circ );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = cs.material.customColor;

		// Fige la couleur de CETTE instance
		m_flaskColor = m_flaskColorPreset;

		// Récupère l'art par couleur (icône/arme)
		const PixelArtColor* art = PixelArtColor_GetByName( FlaskIconName( m_flaskColor ) );
		if ( !art )
			return;

		// Création du "weapon body"
		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdFlask );
		w.isBullet = false;
		m_flaskId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_flaskId, true );
		m_weaponDamage[b2StoreBodyId( m_flaskId )] = 1;

		// Shapes depuis le pixel art
		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );
		float px = kPixelSize;

		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.enableContactEvents = true;

				float lx = ( ( x + 0.5f ) - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				if ( t == Pixel_Sensor )
				{
					sd.isSensor = true;
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdFlask ) );
				}

				sd.material.customColor = art->at( x, y );
				b2CreatePolygonShape( m_flaskId, &sd, &box );
			}

		// Joint (moteur)
		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdFlask;
		jd.base.bodyIdB = m_flaskId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_flaskJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		// Wiring
		m_weaponToJoint[b2StoreBodyId( m_flaskId )] = m_flaskJointId;
		m_weaponOwner[b2StoreBodyId( m_flaskId )] = m_characterIdFlask;
		m_characterWeapon[b2StoreBodyId( m_characterIdFlask )] = m_flaskId;
	}

	void CreateCharacterLance( const b2Vec2& pos )
	{
		// === 1) Character (identique à Sword, noms changés) ===
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = pos;
		bd.linearVelocity = { 5.f, 0.f };
		bd.motionLocks = { false, false, true }; // remplace "fixedRotation" : même que Sword
		bd.linearDamping = 0.0f;
		m_characterIdLance = b2CreateBody( m_worldId, &bd );
		m_characterHP[b2StoreBodyId( m_characterIdLance )] = 100;

		// SKIN sensor
		b2ShapeDef skinSd = b2DefaultShapeDef();
		skinSd.isSensor = true;
		skinSd.enableSensorEvents = true;
		skinSd.filter.categoryBits = CATEGORY_SKIN;
		skinSd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE;
		skinSd.material.customColor = b2_colorKhaki; // strictement comme Sword
		b2Circle circSkin = { { 0, 0 }, kCharacterRadius };
		b2ShapeId skinShapeId = b2CreateCircleShape( m_characterIdLance, &skinSd, &circSkin );
		m_shapeToCharacter[b2StoreShapeId( skinShapeId )] = b2StoreBodyId( m_characterIdLance );
		m_characterSkinShape[b2StoreBodyId( m_characterIdLance )] = skinShapeId;
		m_shapeBaseColor[b2StoreShapeId( skinShapeId )] = skinSd.material.customColor;

		// BODY physique
		b2ShapeDef bodySd = b2DefaultShapeDef();
		bodySd.density = 20.0f;
		bodySd.material = b2DefaultSurfaceMaterial();
		bodySd.material.customColor = b2_colorKhaki;
		bodySd.material.restitution = 1.0f;
		bodySd.filter.categoryBits = CATEGORY_CHARACTER;
		bodySd.filter.maskBits = 0xFFFF;
		bodySd.enableHitEvents = true;
		b2Circle circBody = { { 0, 0 }, kCharacterRadius };
		b2ShapeId bodyCircleId = b2CreateCircleShape( m_characterIdLance, &bodySd, &circBody );
		m_shapeBaseColor[b2StoreShapeId( bodyCircleId )] = bodySd.material.customColor;

		// === 2) Weapon (LANCE) — calqué sur Sword ===
		const PixelArtColor* art = PixelArtColor_GetByName( "LANCE" );
		if ( !art )
			return;

		b2BodyDef w = b2DefaultBodyDef();
		w.type = b2_dynamicBody;
		w.position = b2Body_GetPosition( m_characterIdLance );
		w.isBullet = false;
		m_lanceId = b2CreateBody( m_worldId, &w );
		b2Body_EnableContactEvents( m_lanceId, true );
		m_weaponDamage[b2StoreBodyId( m_lanceId )] = 1; // même valeur que Sword

		std::vector<PixelPhysicsType> map;
		ComputePixelPhysicsMap( *art, map );
		float px = kPixelSize;
		for ( int y = 0; y < art->height; ++y )
			for ( int x = 0; x < art->width; ++x )
			{
				PixelPhysicsType t = map[y * art->width + x];
				if ( t == Pixel_Void )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 0.001f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.enableContactEvents = true;

				float lx = ( float( x ) + 0.5f - art->width * 0.5f ) * px;
				float ly = ( art->height * 0.5f - ( float( y ) + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );

				sd.isSensor = true;
				if ( t == Pixel_Sensor )
				{
					sd.filter.categoryBits = CATEGORY_HITBOX;
					sd.filter.maskBits = CATEGORY_SKIN;
				}
				else
				{
					sd.isSensor = false;
					sd.enableSensorEvents = true;
					sd.enableContactEvents = true;
					sd.filter.categoryBits = CATEGORY_WEAPON;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN;
				}
				sd.material.customColor = art->at( x, y );
				b2CreatePolygonShape( m_lanceId, &sd, &box );
			}

		// === 3) Joint (identique à Sword) ===
		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.base.bodyIdA = m_characterIdLance;
		jd.base.bodyIdB = m_lanceId;
		jd.base.localFrameB.p = { -2.5f, -2.5f };
		jd.enableMotor = true;
		jd.motorSpeed = WithSign( m_motorSpeedAbs, m_globalMotorSign );
		jd.maxMotorTorque = 50.0f;
		m_lanceJointId = b2CreateRevoluteJoint( m_worldId, &jd );

		m_weaponToJoint[b2StoreBodyId( m_lanceId )] = m_lanceJointId;
		m_weaponOwner[b2StoreBodyId( m_lanceId )] = m_characterIdLance;
		m_characterWeapon[b2StoreBodyId( m_characterIdLance )] = m_lanceId;
	}

	// ========= Nettoyage =========

	void SafeDestroyBody( b2BodyId id )
	{
		if ( BodyValid( id ) )
			b2DestroyBody( id );
	}

	

	void DestroyTurret( b2BodyId turret )
	{
		if ( !B2_IS_NON_NULL( turret ) )
			return;

		m_turretOwner.erase( turret );
		m_turretIds.erase( turret );
		m_turretLastShot.erase( turret );

		if ( BodyValid( turret ) )
		{
			b2ShapeId shapes[128];
			int n = b2Body_GetShapes( turret, shapes, 128 );
			for ( int i = 0; i < n; ++i )
			{
				const uint64_t skey = b2StoreShapeId( shapes[i] );
				m_shapeBaseColor.erase( skey );
				m_shapeToCharacter.erase( skey );
			}
		}

		SafeDestroyBody( turret );
	}

	// ========= Itération utilitaire =========

	template <typename F> void ForEachProjectileBody( F&& fn )
	{
		auto doSet = [&]( const BodySet& s ) {
			for ( b2BodyId id : s )
				fn( id );
		};
		doSet( m_projectileArrows );
		doSet( m_projectileFireworks );
		doSet( m_projectileVampireKnives );
		doSet( m_projectileShuriken );
		doSet( m_projectileFrost );
		doSet( m_projectileExplosion );
		doSet( m_projectileElectricStaff );
		doSet( m_projectilePoisonDarts );
		doSet( m_projectileTurrets );
		doSet( m_projectileFlask ); // ⬅️ NEW
	}

	// ========= Aide logique interne =========

	void EraseBodyFromProjectileSets( b2BodyId body )
	{
		auto eraseFromSet = [&]( BodySet& set ) {
			auto it = set.find( body );
			if ( it != set.end() )
				set.erase( it );
		};
		eraseFromSet( m_projectileArrows );
		eraseFromSet( m_projectileFireworks );
		eraseFromSet( m_projectileVampireKnives );
		eraseFromSet( m_projectileShuriken );
		eraseFromSet( m_projectileFrost );
		eraseFromSet( m_projectileExplosion );
		eraseFromSet( m_projectileElectricStaff );
		eraseFromSet( m_projectilePoisonDarts );
		eraseFromSet( m_projectileTurrets );
		eraseFromSet( m_projectileFlask ); // ⬅️ NEW
	}

	void FireTurretProjectile( b2BodyId turretId )
	{
		if ( !B2_IS_NON_NULL( turretId ) )
			return;

		b2Vec2 pos = b2Body_GetPosition( turretId );
		b2Rot rot = b2Body_GetRotation( turretId );
		float angle = b2Atan2( rot.s, rot.c );
		b2Vec2 dir = { cosf( angle ), sinf( angle ) };

		// +20%
		const float baseRadius = 0.66f; // 0.55f * 1.2
		const float barrelHx = 0.30f;	// 0.25f * 1.2
		const float projR = 0.24f;		// 0.20f * 1.2
		const float eps = 0.024f;		// 0.02f * 1.2

		const float spawnDist = baseRadius + 2.0f * barrelHx + projR + eps;
		b2Vec2 spawn = pos + dir * spawnDist;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.gravityScale = 0.0f;
		bd.position = spawn;
		bd.isBullet = false;
		b2BodyId projId = b2CreateBody( m_worldId, &bd );

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 0.00005f;
		sd.material = b2DefaultSurfaceMaterial();
		sd.material.restitution = 1.0f;
		sd.material.customColor = b2_colorOrange;
		sd.enableContactEvents = true;
		sd.enableSensorEvents = true;

		sd.filter.categoryBits = CATEGORY_PROJECTILE;
		sd.filter.maskBits = 0xFFFF;
		sd.filter.groupIndex = MakeTurretGroup( turretId );

		b2Circle c = { { 0, 0 }, projR };
		b2CreateCircleShape( projId, &sd, &c );

		const float projectileSpeed = 10.f;
		b2Body_SetLinearVelocity( projId, dir * projectileSpeed );

		m_weaponDamage[b2StoreBodyId( projId )] = 1;

		RegisterProjectile( projId, ProjectileKind::Turret, turretId );
	}

	void FireBowProjectileFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "BOW_PROJECTILE" );
		if ( !art )
			return;

		b2Transform bowXf = b2Body_GetTransform( m_bowId );
		b2Vec2 tailLocal = ComputePixelLocal( art, kPixelSize, ARROW_TAIL_X, ARROW_TAIL_Y );
		b2Vec2 tipLocal = ComputePixelLocal( art, kPixelSize, ARROW_TIP_X, ARROW_TIP_Y );
		b2Vec2 dir = b2Normalize( tipLocal - tailLocal );
		b2Vec2 shootDir = b2RotateVector( bowXf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( bowXf, tailLocal );

		if ( IsSpawnTooClose( spawnPos, m_characterIdBow, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = bowXf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId arrowId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( arrowId, true );

		const int width = art->width;
		const int height = art->height;
		float px = kPixelSize;

		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.5f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits =
					CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;
				sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdBow ) );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( arrowId, &sd, &box );
			}

		b2Body_SetLinearVelocity( arrowId, b2MulSV( 25.f, shootDir ) );

		m_weaponDamage[b2StoreBodyId( arrowId )] = 1;

		RegisterProjectile( arrowId, ProjectileKind::Arrow, m_characterIdBow );
	}

	void FireFireworkFromCrossbow()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "CROSSBOW_PROJECTILE" );
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_crossbowId );

		// TIP/TAIL → direction de tir
		b2Vec2 tail = ComputePixelLocal( art, kPixelSize, FIREWORK_TAIL_X, FIREWORK_TAIL_Y );
		b2Vec2 tip = ComputePixelLocal( art, kPixelSize, FIREWORK_TIP_X, FIREWORK_TIP_Y );
		b2Vec2 dir = b2Normalize( tip - tail );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawn = b2TransformPoint( xf, tail );

		const int N = std::clamp( m_crossbowVolleyCount, 1, 100 );
		const float spread = 20.f * b2_pi / 180.f;

		for ( int i = 0; i < N; ++i )
		{
			float f = ( N == 1 ) ? 0.f : ( float( i ) - ( N - 1 ) * 0.5f ) / ( ( N - 1 ) * 0.5f );
			float a = f * ( spread * 0.5f ); // ← plus d’offset fixe, comme les autres

			if ( IsSpawnTooClose( spawn, m_characterIdCrossbowBow, 1.0f ) )
				continue;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = spawn;
			bd.rotation = xf.q; // garde l’orientation corps = arme
			bd.isBullet = false;
			bd.gravityScale = 0.0f;
			b2BodyId fireworkId = b2CreateBody( m_worldId, &bd );
			b2Body_EnableContactEvents( fireworkId, true );

			// Fixtures pixel
			float px = kPixelSize;
			const int width = art->width, height = art->height;
			for ( int y = 0; y < height; ++y )
				for ( int x = 0; x < width; ++x )
				{
					uint32_t col = art->at( x, y );
					if ( !col )
						continue;

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.density = 0.00005f;
					sd.material = b2DefaultSurfaceMaterial();
					sd.material.restitution = 1.0f;
					sd.material.customColor = col;
					sd.filter.categoryBits = CATEGORY_PROJECTILE;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET |
										 CATEGORY_KILLZONE;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdCrossbowBow ) );

					float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
					float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
					b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
					b2CreatePolygonShape( fireworkId, &sd, &box );
				}

			// Applique juste le spread autour de shootDir
			float c = std::cos( a ), s = std::sin( a );
			b2Vec2 d2 = { shootDir.x * c - shootDir.y * s, shootDir.x * s + shootDir.y * c };
			b2Body_SetLinearVelocity( fireworkId, b2MulSV( 25.f, d2 ) );

			m_weaponDamage[b2StoreBodyId( fireworkId )] = 2;
			RegisterProjectile( fireworkId, ProjectileKind::Firework, m_characterIdCrossbowBow );
		}
	}

	void FireVampireKnifeProjectile()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "VAMPIRE_KNIFE_PROJECTILE" );
		if ( !art || !B2_IS_NON_NULL( m_vampireKnifeId ) )
			return;

		// Orientation actuelle de l'arme (sert pour le TIR)
		b2Transform xf = b2Body_GetTransform( m_vampireKnifeId );

		// Tir calculé dans le repère "original" du sprite (pas de rotation appliquée ici)
		b2Vec2 tail0 = ComputePixelLocal( art, kPixelSize, VAMPIRE_KNIFE_TAIL_X, VAMPIRE_KNIFE_TAIL_Y );
		b2Vec2 tip0 = ComputePixelLocal( art, kPixelSize, VAMPIRE_KNIFE_TIP_X, VAMPIRE_KNIFE_TIP_Y );
		b2Vec2 dir = b2Normalize( tip0 - tail0 );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawn = b2TransformPoint( xf, tail0 );

		int baseDmg = 0;
		if ( auto it = m_weaponDamage.find( b2StoreBodyId( m_vampireKnifeId ) ); it != m_weaponDamage.end() )
			baseDmg = it->second;

		const int N = 4;
		const float spread = 22.f * b2_pi / 180.f;

		// Rotation d'apparence du SPRITE (tourne tout le body du projectile)
		const float kSpriteYaw = -b2_pi * 0.25f; // -45° (≡ +315°) — ajuste si besoin
		const b2Rot bodyRot = b2MulRot( xf.q, b2MakeRot( kSpriteYaw ) );

		for ( int i = 0; i < N; ++i )
		{
			float f = ( N == 1 ) ? 0.f : ( float( i ) - ( N - 1 ) * 0.5f ) / ( ( N - 1 ) * 0.5f );
			float a = f * ( spread * 0.5f );

			if ( IsSpawnTooClose( spawn, m_characterIdVampireKnife, 1.0f ) )
				continue;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = spawn;
			bd.rotation = bodyRot; // <- le sprite est incliné, fixtures alignées localement
			bd.isBullet = false;
			bd.gravityScale = 0.0f;
			b2BodyId knifeId = b2CreateBody( m_worldId, &bd );
			b2Body_EnableContactEvents( knifeId, true );

			float px = kPixelSize;
			const int width = art->width, height = art->height;
			for ( int y = 0; y < height; ++y )
				for ( int x = 0; x < width; ++x )
				{
					uint32_t col = art->at( x, y );
					if ( !col )
						continue;

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.density = 0.00005f;
					sd.material = b2DefaultSurfaceMaterial();
					sd.material.restitution = 1.0f;
					sd.material.customColor = col;
					sd.filter.categoryBits = CATEGORY_PROJECTILE;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET |
										 CATEGORY_KILLZONE;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdVampireKnife ) );

					float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
					float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;

					// Fixtures non-rotatées localement (AABB locales propres) → c'est le body qui est tourné
					b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
					b2CreatePolygonShape( knifeId, &sd, &box );
				}

			// Direction de tir inchangée (on applique seulement le spread)
			float c = std::cos( a ), s = std::sin( a );
			b2Vec2 d2 = { shootDir.x * c - shootDir.y * s, shootDir.x * s + shootDir.y * c };
			b2Body_SetLinearVelocity( knifeId, b2MulSV( 25.f, d2 ) );

			m_weaponDamage[b2StoreBodyId( knifeId )] = baseDmg;
			RegisterProjectile( knifeId, ProjectileKind::VampireKnife, m_characterIdVampireKnife );
		}
	}

	void FireShurikenFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "SHURIKEN_PROJECTILE" );
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_shurikenId );
		b2Vec2 tail = ComputePixelLocal( art, kPixelSize, SHURIKEN_TAIL_X, SHURIKEN_TAIL_Y );
		b2Vec2 tip = ComputePixelLocal( art, kPixelSize, SHURIKEN_TIP_X, SHURIKEN_TIP_Y );
		b2Vec2 dir = b2Normalize( tip - tail );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tail );

		if ( IsSpawnTooClose( spawnPos, m_characterIdShuriken, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId shurikenProjId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( shurikenProjId, true );

		const int width = art->width, height = art->height;
		float px = kPixelSize;
		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits =
					CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;
				sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdShuriken ) );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( shurikenProjId, &sd, &box );
			}

		const float shurikenSpinRps = 2.0f;											 // rapide pour l’effet “shuriken”
		b2Body_SetAngularVelocity( shurikenProjId, 2.0f * b2_pi * shurikenSpinRps ); // anti-horaire
		b2Body_SetAngularDamping( shurikenProjId, 0.0f );
		b2Body_SetLinearVelocity( shurikenProjId, b2MulSV( 25.f, shootDir ) );
		m_weaponDamage[b2StoreBodyId( shurikenProjId )] = 2;
		RegisterProjectile( shurikenProjId, ProjectileKind::Shuriken, m_characterIdShuriken );
	}

	void FireFrostProjectileFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "FROST_PROJECTILE" ); // ← renommé
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_frostStaffId );
		b2Vec2 tail = ComputePixelLocal( art, kPixelSize, FROST_TAIL_X, FROST_TAIL_Y ); // ← FROST_*
		b2Vec2 tip = ComputePixelLocal( art, kPixelSize, FROST_TIP_X, FROST_TIP_Y );	// ← FROST_*
		b2Vec2 dir = b2Normalize( tip - tail );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tail );

		if ( IsSpawnTooClose( spawnPos, m_characterIdFrostStaff, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId frostId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( frostId, true );

		const int width = art->width, height = art->height;
		float px = kPixelSize;
		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits = CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_KILLZONE;
				sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdFrostStaff ) );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( frostId, &sd, &box );
			}

		const float frostSpinRps = 0.5f;									// léger swirl
		b2Body_SetAngularVelocity( frostId, -2.0f * b2_pi * frostSpinRps ); // négatif = horaire
		b2Body_SetAngularDamping( frostId, 0.0f );							// garder la rotation
		b2Body_SetLinearVelocity( frostId, b2MulSV( 5.f, shootDir ) );
		m_weaponDamage[b2StoreBodyId( frostId )] = 1;
		RegisterProjectile( frostId, ProjectileKind::Frost, m_characterIdFrostStaff );
	}

	void FireExplosionProjectileFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "EXPLOSION_PROJECTILE" );
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_explosionStaffId );
		b2Vec2 tail = ComputePixelLocal( art, kPixelSize, EXPLOSION_TAIL_X, EXPLOSION_TAIL_Y );
		b2Vec2 tip = ComputePixelLocal( art, kPixelSize, EXPLOSION_TIP_X, EXPLOSION_TIP_Y );
		b2Vec2 dir = b2Normalize( tip - tail );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tail );

		if ( IsSpawnTooClose( spawnPos, m_characterIdExplosionStaff, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId explosionId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( explosionId, true );

		const int width = art->width, height = art->height;
		float px = kPixelSize;
		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits =
					CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;
				sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdExplosionStaff ) );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( explosionId, &sd, &box );
			}
		const float spinRps = 2.0f;
		b2Body_SetAngularVelocity( explosionId, -2.0f * b2_pi * spinRps );
		b2Body_SetAngularDamping( explosionId, 0.0f ); // garder la rotation (optionnel, par défaut 0)
		b2Body_SetLinearVelocity( explosionId, b2MulSV( 10.f, shootDir ) );
		m_weaponDamage[b2StoreBodyId( explosionId )] = m_explosionDamage;
		RegisterProjectile( explosionId, ProjectileKind::Explosion, m_characterIdExplosionStaff );
	}

	void FireElectricStaffProjectileFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "ELECTRIC_STAFF_PROJECTILE" ); // ← renommé
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_electricStaffId );
		b2Vec2 tail = ComputePixelLocal( art, kPixelSize, ELECTRIC_STAFF_TAIL_X, ELECTRIC_STAFF_TAIL_Y ); // ← ELECTRIC_STAFF_*
		b2Vec2 tip = ComputePixelLocal( art, kPixelSize, ELECTRIC_STAFF_TIP_X, ELECTRIC_STAFF_TIP_Y );	  // ← ELECTRIC_STAFF_*
		b2Vec2 dir = b2Normalize( tip - tail );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tail );

		if ( IsSpawnTooClose( spawnPos, m_characterIdElectricStaff, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId electricStaffProjId = b2CreateBody( m_worldId, &bd ); // ← nom camelCase
		b2Body_EnableContactEvents( electricStaffProjId, true );

		const int width = art->width, height = art->height;
		float px = kPixelSize;
		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits =
					CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;
				sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdElectricStaff ) );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( electricStaffProjId, &sd, &box );
			}

		b2Body_SetLinearVelocity( electricStaffProjId, b2MulSV( 25.f, shootDir ) );
		m_weaponDamage[b2StoreBodyId( electricStaffProjId )] = 1;
		RegisterProjectile( electricStaffProjId, ProjectileKind::Electric, m_characterIdElectricStaff );
	}

	void FirePoisonDartFromCurrent()
	{
		const PixelArtColor* art = PixelArtColor_GetByName( "POISON_BLOWGUN_PROJECTILE" );
		if ( !art )
			return;

		b2Transform xf = b2Body_GetTransform( m_poisonBlowgunId );

		b2Vec2 tailLocal = ComputePixelLocal( art, kPixelSize, POISON_DART_TAIL_X, POISON_DART_TAIL_Y );
		b2Vec2 tipLocal = ComputePixelLocal( art, kPixelSize, POISON_DART_TIP_X, POISON_DART_TIP_Y );
		b2Vec2 dir = b2Normalize( tipLocal - tailLocal );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tailLocal );

		if ( IsSpawnTooClose( spawnPos, m_characterIdPoisonBlowgun, 1.0f ) )
			return;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false;
		bd.gravityScale = 0.0f;
		b2BodyId dartId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( dartId, true );

		const int width = art->width, height = art->height;
		float px = kPixelSize;
		for ( int y = 0; y < height; ++y )
			for ( int x = 0; x < width; ++x )
			{
				uint32_t col = art->at( x, y );
				if ( !col )
					continue;

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.enableContactEvents = true;
				sd.enableSensorEvents = true;
				sd.density = 0.00005f;
				sd.material = b2DefaultSurfaceMaterial();
				sd.material.restitution = 1.0f;
				sd.material.customColor = col;
				sd.filter.categoryBits = CATEGORY_PROJECTILE;
				sd.filter.maskBits =
					CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;

				// CLÉ : identique à l’arme, basé sur le même owner (le perso blowgun)
				sd.filter.groupIndex = MakeTurretGroup( m_characterIdPoisonBlowgun );

				float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
				float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
				b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2MakeRot( 0 ) );
				b2CreatePolygonShape( dartId, &sd, &box );
			}

		b2Body_SetLinearVelocity( dartId, b2MulSV( 25.f, shootDir ) );

		m_weaponDamage[b2StoreBodyId( dartId )] = 1;

		RegisterProjectile( dartId, ProjectileKind::PoisonDart, m_characterIdPoisonBlowgun );
	}

	void FireFlaskProjectileFromCurrent()
	{
		if ( !B2_IS_NON_NULL( m_flaskId ) || !B2_IS_NON_NULL( m_characterIdFlask ) )
			return;

		// Sprite projectile par couleur (fallback générique si manquant)
		const PixelArtColor* art = PixelArtColor_GetByName( FlaskProjectileName( m_flaskColor ) );
		if ( !art )
			art = PixelArtColor_GetByName( "FLASK_PROJECTILE" );

		// Transform & direction (comme Bow/Shuriken)
		b2Transform xf = b2Body_GetTransform( m_flaskId );
		b2Vec2 tailLocal = { 0.f, 0.f }, tipLocal = { kPixelSize, 0.f };
		if ( art )
		{
			tailLocal = ComputePixelLocal( art, kPixelSize, FLASK_TAIL_X, FLASK_TAIL_Y );
			tipLocal = ComputePixelLocal( art, kPixelSize, FLASK_TIP_X, FLASK_TIP_Y );
		}
		b2Vec2 dir = b2Normalize( tipLocal - tailLocal );
		b2Vec2 shootDir = b2RotateVector( xf.q, dir );
		b2Vec2 spawnPos = b2TransformPoint( xf, tailLocal );

		if ( IsSpawnTooClose( spawnPos, m_characterIdFlask, 1.0f ) )
			return;

		// Body projectile
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = spawnPos;
		bd.rotation = xf.q;
		bd.isBullet = false; // comme les autres projectiles
		bd.gravityScale = 0.0f;
		b2BodyId projId = b2CreateBody( m_worldId, &bd );
		b2Body_EnableContactEvents( projId, true );

		int shapeCount = 0;

		if ( art )
		{
			// Fixtures pixel (copie le pattern Bow/Shuriken)
			const int width = art->width, height = art->height;
			const float px = kPixelSize;

			for ( int y = 0; y < height; ++y )
				for ( int x = 0; x < width; ++x )
				{
					uint32_t col = art->at( x, y );
					if ( !col )
						continue;

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.enableContactEvents = true;
					sd.enableSensorEvents = true;
					sd.density = 0.00005f;
					sd.material = b2DefaultSurfaceMaterial();
					sd.material.restitution = 1.5f; // idem Bow
					sd.material.customColor = col;
					sd.filter.categoryBits = CATEGORY_PROJECTILE;
					sd.filter.maskBits = CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET |
										 CATEGORY_KILLZONE;
					sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdFlask ) ); // même pattern que Bow

					float lx = ( ( x + 0.5f ) - width * 0.5f ) * px;
					float ly = ( height * 0.5f - ( y + 0.5f ) ) * px;
					b2Polygon box = b2MakeOffsetBox( px * 0.5f, px * 0.5f, { lx, ly }, b2Rot_identity );
					b2CreatePolygonShape( projId, &sd, &box );
					++shapeCount;
				}
		}
		else
		{
			// Fallback géométrique : petit disque (même style que la tourelle)
			b2ShapeDef sd = b2DefaultShapeDef();
			sd.enableContactEvents = true;
			sd.enableSensorEvents = true;
			sd.density = 0.001f;
			sd.material = b2DefaultSurfaceMaterial();
			sd.material.restitution = 0.6f;
			sd.material.customColor = 0xFFAA55;
			sd.filter.categoryBits = CATEGORY_PROJECTILE;
			sd.filter.maskBits =
				CATEGORY_WEAPON | CATEGORY_PROJECTILE | CATEGORY_SKIN | CATEGORY_WALL | CATEGORY_TURRET | CATEGORY_KILLZONE;
			sd.filter.groupIndex = -int( b2StoreBodyId( m_characterIdFlask ) );

			b2Circle c = { { 0.f, 0.f }, 0.25f };
			b2CreateCircleShape( projId, &sd, &c );
			shapeCount = 1;
		}

		if ( shapeCount == 0 )
		{
			SafeDestroyBody( projId );
			return;
		}

		// Vitesse + enregistrement
		b2Body_SetLinearVelocity( projId, b2MulSV( 25.f, shootDir ) );
		m_weaponDamage[b2StoreBodyId( projId )] = 1;
		RegisterProjectile( projId, ProjectileKind::Flask, m_characterIdFlask );
	}
};

static int WeaponsBallsVSReg = RegisterSample( "VS Games", "WeaponsBallsVS", WeaponsBallsVS::Create );

class HumanVS : public Sample
{
public:
	explicit HumanVS( SampleContext* context )
		: Sample( context )
	{
		m_context->camera.center = { 0.0f, 0.0f };
		m_context->camera.zoom = 12.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = 1.3f;
		shapeDef.material.friction = 0.1f;

		{
			b2Segment segment = { { -10.0f, -10.0f }, { 10.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}
		{
			b2Segment segment = { { 10.0f, -10.0f }, { 10.0f, 10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}
		{
			b2Segment segment = { { 10.0f, 10.0f }, { -10.0f, 10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}
		{
			b2Segment segment = { { -10.0f, 10.0f }, { -10.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2Circle circle = { { 0.0f, 0.0f }, 2.0f };
		shapeDef.material.restitution = 2.0f;
		b2CreateCircleShape( groundId, &shapeDef, &circle );
	}

	void Step() override
	{
		if ( m_humanCount < 5 && m_countDown <= 0.0f )
		{
			float jointFrictionTorque = 0.0f;
			float jointHertz = 1.0f;
			float jointDampingRatio = 0.1f;

			CreateHuman( m_humans + m_humanCount, m_worldId, { 0.0f, 5.0f }, 1.0f, jointFrictionTorque, jointHertz,
						 jointDampingRatio, 1, nullptr, true );

			// Human_SetVelocity(m_humans + m_humanCount, { 10.0f - 5.0f * m_humanCount, -20.0f + 5.0f * m_humanCount });

			m_countDown = 2.0f;
			m_humanCount += 1;
		}

		float timeStep = 1.0f / 60.0f;
		b2CosSin cs1 = b2ComputeCosSin( 0.5f * m_time );
		b2CosSin cs2 = b2ComputeCosSin( m_time );
		float gravity = 10.0f;
		b2Vec2 gravityVec = { gravity * cs1.sine, gravity * cs2.cosine };
		DrawLine( m_draw, b2Vec2_zero, b2Vec2{ 3.0f * cs1.sine, 3.0f * cs2.cosine }, b2_colorWhite );
		m_time += timeStep;
		m_countDown -= timeStep;
		b2World_SetGravity( m_worldId, gravityVec );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new HumanVS( context );
	}

	Human m_humans[5] = {};
	int m_humanCount = 0;
	float m_countDown = 0.0f;
	float m_time = 0.0f;
};

static int sampleHumanVS = RegisterSample( "VS Games", "Human VS", HumanVS::Create );
