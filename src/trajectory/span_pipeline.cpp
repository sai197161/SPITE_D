#include "trajectory/span_pipeline.hpp"

namespace spite_d {

SpanPipeline::SpanPipeline(const Predictor& predictor, ValidityServer& server,
                           Params params)
    : m_predictor(predictor), m_server(server), m_params(params) {}

void SpanPipeline::Update(const std::vector<TrackedObstacle>& tracks) {
  ++m_stats.frames;

  std::vector<PredictedTrajectory> rebuilds;
  for (const TrackedObstacle& track : tracks) {
    ++m_stats.observations;

    const auto it = m_spans.find(track.id);
    if (it != m_spans.end() && it->second.span.Conforms(track) &&
        !it->second.span.NeedsRefresh(track.stamp)) {
      ++m_stats.conforming;

      // Sliced mode: lapse slices the clock has passed. Bookkeeping
      // only -- edges blocked solely by the obstacle's past heal here
      // without any geometry rebuild.
      if (m_params.slices > 1) {
        TrackedSpan& ts = it->second;
        const size_t active =
            SliceIndexAt(track.stamp, ts.span.StartTime(), ts.span.EndTime(),
                         m_params.slices);
        if (active > ts.firstActiveSlice) {
          m_server.ExpireSlices(track.id, active);
          m_stats.expiries += active - ts.firstActiveSlice;
          ts.firstActiveSlice = active;
        }
      }
      continue;
    }

    PredictedTrajectory traj =
        m_predictor.Predict(track, m_params.horizon, m_params.dt);

    if (m_params.slices > 1) {
      m_server.UpdateSlices(track.id, SliceTrajectory(traj, m_params.slices));
    } else {
      rebuilds.push_back(traj);
    }
    m_spans.insert_or_assign(track.id,
                             TrackedSpan{Span(std::move(traj), m_params.span), 0});
    ++m_stats.rebuilds;
  }

  // Only rebuilt slots get marked dirty; ValidityServer::Update runs
  // the RGG pass for those alone. On fully-conforming frames no slot
  // changed, so even the reconciliation sweep is skippable.
  if (!rebuilds.empty()) m_server.Update(rebuilds);
}

void SpanPipeline::Forget(int32_t obstacleId) {
  m_spans.erase(obstacleId);
  m_server.Forget(obstacleId);
}

const Span* SpanPipeline::GetSpan(int32_t obstacleId) const {
  const auto it = m_spans.find(obstacleId);
  return it == m_spans.end() ? nullptr : &it->second.span;
}

}  // namespace spite_d
