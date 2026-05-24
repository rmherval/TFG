#!/usr/bin/env python3
"""
Debug script específico para investigar el segmento 3.
Muestra toda la información sobre goal 3 y las poses cercanas.
"""

import math
from pathlib import Path
from simple_analyzer import SimpleNavigationAnalyzer

def main():
    analyzer = SimpleNavigationAnalyzer('./bags/recorrido_largo_nav2')
    
    # Obtener goals
    unique_goals = analyzer._extract_unique_goals()
    print(f"Total goals: {len(unique_goals)}\n")
    
    for i, goal in enumerate(unique_goals):
        ts, x, y, z = goal
        goal_odom = analyzer._map_to_odom((x, y, z))
        print(f"Goal {i+1}:")
        print(f"  Map:  x={x:.4f}, y={y:.4f}, z={z:.4f}")
        print(f"  Odom: x={goal_odom[0]:.4f}, y={goal_odom[1]:.4f}, z={goal_odom[2]:.4f}")
        print()
    
    # Enfocarse en goal 3 (segmento 3)
    print("\n" + "="*80)
    print("DEBUG SEGMENTO 3: Goal 2 → Goal 3")
    print("="*80)
    
    goal2 = unique_goals[1]  # Goal 2 (índice 1)
    goal3 = unique_goals[2]  # Goal 3 (índice 2)
    
    goal2_odom = analyzer._map_to_odom(goal2[1:])
    goal3_odom = analyzer._map_to_odom(goal3[1:])
    
    print(f"\nGoal 2 (map):  x={goal2[1]:.4f}, y={goal2[2]:.4f}")
    print(f"Goal 2 (odom): x={goal2_odom[0]:.4f}, y={goal2_odom[1]:.4f}")
    print(f"\nGoal 3 (map):  x={goal3[1]:.4f}, y={goal3[2]:.4f}")
    print(f"Goal 3 (odom): x={goal3_odom[0]:.4f}, y={goal3_odom[1]:.4f}")
    
    # Obtener todas las poses
    odom_messages = analyzer.messages_by_topic.get("/odom", [])
    poses = []
    for timestamp, msg in odom_messages:
        pose = analyzer._get_pose(msg)
        if pose:
            poses.append((timestamp, *pose))
    
    print(f"\nTotal poses en /odom: {len(poses)}")
    print(f"Timestamps: {poses[0][0]} a {poses[-1][0]} (duración: {(poses[-1][0] - poses[0][0])/1e9:.1f}s)")
    print(f"Goal 2 timestamp: {goal2[0]}")
    print(f"Goal 3 timestamp: {goal3[0]}")
    
    # Encontrar el índice de inicio (cerca de goal 2)
    start_idx = None
    for i, (_, x, y, z) in enumerate(poses):
        dist = analyzer._distance_2d((x, y, z), goal2_odom)
        if dist < analyzer.ARRIVAL_TOLERANCE:
            start_idx = i
            print(f"\n✓ Start índice (cerca de Goal 2): {i}, distancia: {dist:.3f}m")
            print(f"  Pose en start_idx: x={x:.4f}, y={y:.4f}, z={z:.4f}")
            break
    
    if start_idx is None:
        start_idx = 0
        print(f"\n⚠️  No se encontró start_idx, usando 0")
    
    # Buscar el índice de fin (cerca de goal 3)
    end_idx = None
    closest_dist = float('inf')
    closest_idx = None
    
    print(f"\nBuscando end_idx (poses cercanas a Goal 3) desde índice {start_idx}:")
    print(f"Tolerancia de llegada: {analyzer.ARRIVAL_TOLERANCE}m\n")
    
    for i in range(start_idx, len(poses)):
        _, x, y, z = poses[i]
        dist = analyzer._distance_2d((x, y, z), goal3_odom)
        
        # Rastrear la distancia más cercana
        if dist < closest_dist:
            closest_dist = dist
            closest_idx = i
        
        # Mostrar poses cada 50 índices o si están muy cerca
        if (i - start_idx) % 50 == 0 or dist < 1.0:
            marker = ""
            if dist < analyzer.ARRIVAL_TOLERANCE:
                marker = " ← DENTRO DE TOLERANCIA"
                if end_idx is None:
                    end_idx = i
            print(f"  Índice {i}: x={x:.4f}, y={y:.4f} -> dist a Goal 3: {dist:.4f}m{marker}")
        
        if end_idx is None and dist < analyzer.ARRIVAL_TOLERANCE:
            end_idx = i
    
    print(f"\n✓ Distancia más cercana a Goal 3: {closest_dist:.4f}m en índice {closest_idx}")
    if end_idx is not None:
        print(f"✓ end_idx encontrado: {end_idx}")
    else:
        print(f"✗ end_idx NO encontrado (ninguna pose dentro de {analyzer.ARRIVAL_TOLERANCE}m)")
        end_idx = len(poses) - 1
    
    # Mostrar último bloque de poses del segmento
    print(f"\n" + "-"*80)
    print("Últimas 10 poses del segmento 3:")
    print("-"*80)
    
    segment_poses = poses[start_idx:end_idx+1]
    for idx, (ts, x, y, z) in enumerate(segment_poses[-10:]):
        dist = analyzer._distance_2d((x, y, z), goal3_odom)
        actual_idx = start_idx + len(segment_poses) - 10 + idx
        print(f"Índice {actual_idx}: x={x:.4f}, y={y:.4f}, z={z:.4f} -> dist a Goal 3: {dist:.4f}m")
    
    # Calcular métricas
    print(f"\n" + "-"*80)
    print("Cálculo de precisión:")
    print("-"*80)
    
    precision_pose = segment_poses[-1][1:]
    print(f"Última pose del segmento: x={precision_pose[0]:.4f}, y={precision_pose[1]:.4f}, z={precision_pose[2]:.4f}")
    print(f"Goal 3 (odom):            x={goal3_odom[0]:.4f}, y={goal3_odom[1]:.4f}, z={goal3_odom[2]:.4f}")
    
    precision = analyzer._distance_2d(precision_pose, goal3_odom)
    print(f"Precisión calculada: {precision:.4f}m")
    print(f"Dentro de tolerancia ({analyzer.ARRIVAL_TOLERANCE}m): {precision <= analyzer.ARRIVAL_TOLERANCE}")
    print(f"Dentro de umbral adaptativo ({analyzer.ARRIVAL_TOLERANCE * 1.5}m): {precision <= analyzer.ARRIVAL_TOLERANCE * 1.5}")
    
    # Información sobre TF
    print(f"\n" + "-"*80)
    print("Información de transformación:")
    print("-"*80)
    tf_map_odom = analyzer._extract_tf_transform("map", "odom")
    if tf_map_odom:
        print(f"✓ TF map->odom encontrada:")
        print(f"  tx={tf_map_odom[0]:.4f}, ty={tf_map_odom[1]:.4f}, yaw={tf_map_odom[2]:.4f}")
    else:
        print(f"✗ TF map->odom NO encontrada")

if __name__ == "__main__":
    main()
