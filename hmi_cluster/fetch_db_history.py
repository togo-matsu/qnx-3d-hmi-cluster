#!/usr/bin/env python3
import csv
import os
import sys

OUTPUT_CSV = "/tmp/odom_history.csv"

def main():
    try:
        import psycopg2
    except ImportError as e:
        print(f"Warning: psycopg2 not available ({e}); skipping DB fetch.", file=sys.stderr)
        return 0

    host = os.getenv("PG_HOST", "10.0.0.100")
    port = int(os.getenv("PG_PORT", "30432"))
    dbname = os.getenv("PG_DB", "iot_data")
    user = os.getenv("PG_USER", "mqtt_user")
    password = os.getenv("PG_PASS", "mqtt_password")

    # 座標が変化した（移動した）レコードのみを直近500点抽出する
    query = """
        WITH moved AS (
            SELECT
                id,
                (payload->>'odom_x')::float AS odom_x,
                (payload->>'odom_y')::float AS odom_y,
                LAG((payload->>'odom_x')::float) OVER (ORDER BY id) AS prev_x,
                LAG((payload->>'odom_y')::float) OVER (ORDER BY id) AS prev_y
            FROM telemetry
            WHERE device_id = 'sdv-ros2'
              AND payload->>'odom_x' IS NOT NULL
              AND payload->>'odom_y' IS NOT NULL
        )
        SELECT odom_x, odom_y FROM (
            SELECT id, odom_x, odom_y
            FROM moved
            WHERE prev_x IS NULL
               OR ABS(odom_x - prev_x) > 0.01
               OR ABS(odom_y - prev_y) > 0.01
            ORDER BY id DESC
            LIMIT 500
        ) sub
        ORDER BY id ASC;
    """

    try:
        conn = psycopg2.connect(
            host=host,
            port=port,
            dbname=dbname,
            user=user,
            password=password,
            connect_timeout=2
        )
        with conn:
            with conn.cursor() as cur:
                cur.execute(query)
                rows = cur.fetchall()
        conn.close()

        with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["odom_x", "odom_y"])
            for x, y in rows:
                writer.writerow([x, y])

        print(f"Fetched {len(rows)} moved trajectory points to {OUTPUT_CSV}.")
    except Exception as e:
        print(f"Warning: failed to fetch DB history ({e}); continuing.", file=sys.stderr)

    return 0

if __name__ == "__main__":
    sys.exit(main())
